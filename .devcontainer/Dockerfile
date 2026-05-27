# ── Claude Code Devcontainer ──────────────────────────────────────────────────
#
# LAYER STRATEGY:
#   1. base      → OS + core utilities
#   2. runtime   → your language runtime(s) — swap/extend this for your stack
#   3. devtools  → project tooling (git, build tools, etc.)
#   4. claude    → Node.js + Claude Code CLI  ← always on top
#
# This ordering means:
#   - Claude Code updates don't invalidate your expensive runtime layers
#   - You can swap the runtime layer without touching the Claude layer
#   - Multiple projects can share a common "layers 1-3" base image
# ─────────────────────────────────────────────────────────────────────────────

# ── Layer 1: Base OS ──────────────────────────────────────────────────────────
FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04 AS base

# Non-interactive installs
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    wget \
    git \
    git-lfs \
    ca-certificates \
    gnupg \
    lsb-release \
    unzip \
    jq \
    ripgrep \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# ── Layer 2: Language Runtime ─────────────────────────────────────────────────
# Replace or add runtimes here for your project.
# Examples commented out below — uncomment what you need.

# --- Python ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# --- Go (uncomment if needed) ---
# ARG GO_VERSION=1.22.3
# RUN curl -fsSL "https://go.dev/dl/go${GO_VERSION}.linux-amd64.tar.gz" | tar -C /usr/local -xz
# ENV PATH="/usr/local/go/bin:${PATH}"

# --- Rust (uncomment if needed) ---
# RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
# ENV PATH="/root/.cargo/bin:${PATH}"

# ── Layer 3: Dev Tools ────────────────────────────────────────────────────────
# Docker CLI (for projects that need to talk to a Docker daemon via socket mount) (uncomment if needed)
# RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
#     | gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg \
#   && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] \
#     https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" \
#     > /etc/apt/sources.list.d/docker.list \
#   && apt-get update && apt-get install -y --no-install-recommends docker-ce-cli \
#   && rm -rf /var/lib/apt/lists/*

# ── Layer 4: Node.js + Claude Code ───────────────────────────────────────────
# Claude Code requires Node.js 18+. We install it here, on top of everything
# else, so updates to Claude Code only invalidate this final layer.

ARG NODE_VERSION=20
RUN curl -fsSL https://deb.nodesource.com/setup_${NODE_VERSION}.x | bash - \
  && apt-get install -y nodejs \
  && rm -rf /var/lib/apt/lists/*

# Install Claude Code globally
RUN npm install -g @anthropic-ai/claude-code

# ── User setup ────────────────────────────────────────────────────────────────
# The base image provides a non-root "vscode" user. We use that for Claude Code
# so credentials go into /home/vscode/.claude rather than /root/.claude.

USER vscode

# Claude stores auth + settings here. This directory will be backed by a
# named volume (see docker-compose.yml) so it survives container rebuilds.
RUN mkdir -p /home/vscode/.claude

# Verify install
RUN claude --version

WORKDIR /workspace
