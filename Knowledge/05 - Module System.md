# The Paparazzi Module System

## What is a Module?

A module is a self-contained unit of functionality described by an XML file in `conf/modules/`. The XML file declares:
- What `.c`/`.h` files to compile
- Which `#define` / `configure` macros to set
- Which GCS settings panel to expose
- Dependencies on other modules

## Key Directories

```
conf/modules/             ← module XML definitions (one per module)
conf/settings/modules/    ← GCS settings panel XMLs (referenced by conf_*.xml)
sw/airborne/modules/      ← module C source (most generic modules)
sw/airborne/firmwares/rotorcraft/stabilization/  ← firmware-specific stabilization
sw/airborne/firmwares/rotorcraft/guidance/       ← firmware-specific guidance
```

## Module XML Anatomy

```xml
<module name="stabilization_indi" dir="stabilization" task="control">
  <doc>...</doc>

  <settings>                      <!-- GCS-visible tuning panel -->
    <dl_settings>
      <dl_settings NAME="indi">
        <dl_setting var="indi_gains.att.p" .../>
      </dl_settings>
    </dl_settings>
  </settings>

  <dep>
    <depends>stabilization_rotorcraft,@attitude_command,wls</depends>
    <provides>commands</provides>
  </dep>

  <header>
    <file name="stabilization_attitude_quat_indi.h"/>
  </header>

  <init fun="stabilization_indi_init()"/>   <!-- called at boot -->

  <makefile target="ap|nps" firmware="rotorcraft">
    <file name="stabilization_indi.c" dir="$(SRC_FIRMWARE)/stabilization"/>
    <define name="INDI_OUTPUTS" value="$(INDI_OUTPUTS)"/>
    <define name="INDI_NUM_ACT" value="$(INDI_NUM_ACT)"/>
  </makefile>
</module>
```

## Module Type Variants

The airframe selects a variant with the `type` attribute:

```xml
<module name="stabilization" type="indi"/>
```

This causes the build system to look for `conf/modules/stabilization_indi.xml`.  
The naming convention is `{name}_{type}.xml`.

## Module Dependency Graph (INDI stack)

```
stabilization_indi
    └─ depends: stabilization_rotorcraft, wls

guidance_indi
    └─ depends: guidance_indi_quadrotor
         └─ depends: guidance_indi_base
               └─ depends: navigation, guidance_rotorcraft
                    └─ provides: guidance, attitude_command
```

## How Modules Get Called at Runtime

The build system generates `sw/airborne/generated/modules.c` and `modules.h` which contain `modules_init()` and `modules_periodic_task()`. These call each module's `<init>` and `<periodic>` functions in order.

The frequency of the periodic call is set by `PERIODIC_FREQUENCY` (1000 Hz for ANTON).

## Adding a New Module

1. Create `conf/modules/my_controller.xml`
2. Write the `.c`/`.h` in the appropriate `sw/airborne/` subtree
3. Add `<module name="my_controller"/>` to the airframe XML
4. Add it to `settings_modules` in `conf_enac.xml` if you want a GCS panel

## See Also

- [[04 - Airframe XML Configuration]]
- [[06 - Modifying ANTON Stabilization]]
