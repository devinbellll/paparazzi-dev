#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# ── Phase 1: Preserve Docker's internal DNS before any flushing ───────────────
# Docker uses 127.0.0.11 as its embedded DNS resolver. The NAT rules that
# redirect container DNS queries to it must survive the iptables flush below.
DOCKER_DNS_RULES=$(iptables-save -t nat | grep "127\.0\.0\.11" || true)

# ── Phase 2: Flush all existing rules ─────────────────────────────────────────
iptables -F
iptables -X
iptables -t nat -F
iptables -t nat -X
iptables -t mangle -F
iptables -t mangle -X

ipset destroy allowed-domains 2>/dev/null || true

# ── Phase 3: Restore Docker DNS NAT rules ─────────────────────────────────────
if [[ -n "$DOCKER_DNS_RULES" ]]; then
  echo "Restoring Docker DNS NAT rules..."
  iptables -t nat -N DOCKER_OUTPUT     2>/dev/null || true
  iptables -t nat -N DOCKER_POSTROUTING 2>/dev/null || true
  # xargs splits on whitespace regardless of shell IFS (which is \n\t here)
  echo "$DOCKER_DNS_RULES" | xargs -L 1 iptables -t nat
else
  echo "No Docker DNS NAT rules to restore."
fi

# ── Phase 4: Allow loopback, DNS, SSH before setting DROP policy ──────────────
iptables -A INPUT  -i lo -j ACCEPT
iptables -A OUTPUT -o lo -j ACCEPT

# DNS (UDP + TCP) — both directions
iptables -A OUTPUT -p udp --dport 53 -j ACCEPT
iptables -A OUTPUT -p tcp --dport 53 -j ACCEPT
iptables -A INPUT  -p udp --sport 53 -j ACCEPT
iptables -A INPUT  -p tcp --sport 53 -m state --state ESTABLISHED -j ACCEPT

# SSH outbound (e.g. git over SSH), inbound established only
iptables -A OUTPUT -p tcp --dport 22 -j ACCEPT
iptables -A INPUT  -p tcp --sport 22 -m state --state ESTABLISHED -j ACCEPT

# ── Phase 5: Build the allowed-domains ipset ──────────────────────────────────
ipset create allowed-domains hash:net

# — GitHub IP ranges (web + api + git arrays from api.github.com/meta) --------
echo "Fetching GitHub IP ranges..."
gh_meta=$(curl -sf --connect-timeout 10 https://api.github.com/meta)
if [[ -z "$gh_meta" ]]; then
  echo "ERROR: Failed to fetch https://api.github.com/meta" >&2
  exit 1
fi
if ! echo "$gh_meta" | jq -e '.web and .api and .git' >/dev/null 2>&1; then
  echo "ERROR: GitHub meta response missing required fields (web/api/git)" >&2
  exit 1
fi

echo "Processing GitHub CIDR ranges..."
while IFS= read -r cidr; do
  # Validate: must look like an IPv4 CIDR
  if [[ ! "$cidr" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}/[0-9]{1,2}$ ]]; then
    echo "ERROR: Invalid CIDR from GitHub meta: $cidr" >&2
    exit 1
  fi
  echo "  Adding GitHub range $cidr"
  ipset add allowed-domains "$cidr" 2>/dev/null || true   # duplicate ranges OK
done < <(
  echo "$gh_meta" \
    | jq -r '(.web + .api + .git) | map(select(test("^[0-9]"))) | unique[]' \
    | { command -v aggregate &>/dev/null && aggregate -q || cat; }
)

# — Helper: resolve a domain and add all its IPs to the ipset -----------------
_add_domain() {
  local domain="$1"
  local required="${2:-true}"

  echo "Resolving $domain..."
  local ips
  ips=$(dig +short "$domain" | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' || true)

  if [[ -z "$ips" ]]; then
    if [[ "$required" == "true" ]]; then
      echo "ERROR: Failed to resolve required domain: $domain" >&2
      exit 1
    else
      echo "WARNING: Could not resolve $domain (optional — skipping)" >&2
      return 0
    fi
  fi

  while IFS= read -r ip; do
    echo "  Adding $ip ($domain)"
    ipset add allowed-domains "$ip" 2>/dev/null || true
  done <<< "$ips"
}

# Required domains — any failure aborts
_add_domain "api.anthropic.com"
_add_domain "sentry.io"
_add_domain "registry.npmjs.org"
_add_domain "pypi.org"
_add_domain "files.pythonhosted.org"
_add_domain "github.com"
_add_domain "api.github.com"

# Ubuntu apt repositories — needed for apt-get install inside the container
_add_domain "archive.ubuntu.com"
_add_domain "security.ubuntu.com"
_add_domain "esm.ubuntu.com"

# Optional domains — DNS resolution failures are warn-only (known issues)
_add_domain "statsig.com"            "false"
_add_domain "statsig.anthropic.com"  "false"   # known DNS bug: github.com/anthropics/claude-code#55623

# — Host gateway (VS Code port forwarding) ------------------------------------
echo "Detecting host gateway..."
HOST_IP=$(ip route | awk '/default/ {print $3; exit}')
if [[ -z "$HOST_IP" ]]; then
  echo "ERROR: Could not detect host gateway IP" >&2
  exit 1
fi
echo "  Host gateway: $HOST_IP"
iptables -A INPUT  -s "$HOST_IP" -j ACCEPT
iptables -A OUTPUT -d "$HOST_IP" -j ACCEPT

# ── Phase 6: Set default DROP, then allow established + ipset ─────────────────
iptables -P INPUT   DROP
iptables -P FORWARD DROP
iptables -P OUTPUT  DROP

iptables -A INPUT  -m state --state ESTABLISHED,RELATED -j ACCEPT
iptables -A OUTPUT -m state --state ESTABLISHED,RELATED -j ACCEPT

iptables -A OUTPUT -m set --match-set allowed-domains dst -j ACCEPT

# REJECT (not silent DROP) so callers get immediate feedback on blocked traffic
iptables -A OUTPUT -j REJECT --reject-with icmp-admin-prohibited

# ── Phase 7: Verify ───────────────────────────────────────────────────────────
echo ""
echo "Verifying firewall rules..."

if curl --connect-timeout 5 -sf https://example.com >/dev/null 2>&1; then
  echo "ERROR: Firewall verification FAILED — was able to reach https://example.com" >&2
  exit 1
fi
echo "  PASS: https://example.com is blocked (expected)"

if ! curl --connect-timeout 10 -sf https://api.github.com/zen >/dev/null 2>&1; then
  echo "ERROR: Firewall verification FAILED — unable to reach https://api.github.com" >&2
  exit 1
fi
echo "  PASS: https://api.github.com is reachable (expected)"

echo ""
echo "Firewall configuration complete."
