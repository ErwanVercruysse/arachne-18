# Regenerating the v4 robot

Editable geometry lives in `src/v4_leg.py`, `v4_body.py`, `v4_dorsal.py`, `v4_hardware.py` and `v4_servo.py`. `build_v4.py` assembles them. The older entry point `build_cad.py` now calls the v4 builder. `cad/parameters.json` is an output summary; editing it alone does not rebuild geometry.

## CAD build

CadQuery 2.8 needs a compatible Python/OCP environment. A Linux Docker environment avoids native OCP compatibility issues:

```sh
docker build -t arachne-cad -f src/Dockerfile .
docker run --rm -v "$PWD:/workspace" -w /workspace arachne-cad python src/build_v4.py
docker run --rm -v "$PWD:/workspace" -w /workspace arachne-cad python src/check_v4.py
docker run --rm -v "$PWD:/workspace" -w /workspace arachne-cad python src/check_v4_leg.py cad/reference/ST3215.step
docker run --rm -v "$PWD:/workspace" -w /workspace arachne-cad python src/check_v4_body.py cad/reference/ST3215.step
docker run --rm -v "$PWD:/workspace" -w /workspace arachne-cad python src/check_v4_installation.py
```

Alternatively install `src/requirements-cad.txt` in a compatible Python 3.11 environment and run those Python commands directly. The build writes only into ignored `build/v4/`, including STEP, oriented STL and BREP instance cache. It does not commit, upload or overwrite the released print files.

`cad/reference/validated_v4/` freezes the earlier checked component solids. The equivalence check allows reuse of their fit evidence and catches accidental changes during whole-robot integration. If intentionally changing a baseline component, review and rerun its fit/motion checks before explicitly updating that baseline; do not silently weaken the comparison.

## Export and validate simulation

From the CAD environment:

```sh
python src/export_v4.py
python src/render_gallery.py
```

This writes the current `simulation/` geometry, meshes, mass properties and joint map. It preserves BAM parameters and runtime code. STLs in `simulation/meshes/` use **metres and link-local coordinates**; printable STLs use **millimetres and bed orientation**. Never exchange them.

In the Python 3.12 simulation environment:

```sh
python simulation/validate.py
python simulation/grounded_demo.py --headless --seconds 8 --report simulation/grounded_validation.json
python training/build_model.py
python -m unittest discover -s training -p 'test_*.py'
python src/render_simulation.py
```

Rendering additionally needs Pillow and a usable OpenGL backend. No training job runs in these commands. Keep collision descriptions, mass assumptions and joint geometry synchronized when editing the design.

## Slice and publish locally

`src/audit_slicer_v4.py` invokes the installed macOS Bambu Studio CLI. It resolves profile inheritance and uses an A1 0.4 mm profile as a geometry proxy, because the CLI build does not expose a usable H2D profile. The supplied `v4_slicer_profile.json` is the audit process configuration, not a machine-ready H2D project. Reslice the release files on H2D.

After all CAD reports and `build/v4/slicer_audit.json` exist:

```sh
python src/package_v4.py
```

This replaces the canonical `cad/` and `print/` generated files, copies reports and builds the print ZIP. It removes obsolete current-release print/mesh files. Review the diff before committing. Historical v3 reports are explicitly separated in `validation/legacy-v3/`.

## Onshape maintenance

Import the complete assembly STEP into the project document with **flatten assemblies** enabled, to create a positioned 336-part Part Studio. The [official import documentation](https://onshape-public.github.io/docs/api-adv/translation/) describes API upload options. The source Part Studio contains imported solids, not the CadQuery parametric feature tree.

`create_onshape_v4.py` creates/resumes a native assembly from the specified Part Studio and writes a new mate plan. It checks all names against the CAD instance list and fixes the chassis on insertion. It uses `build/onshape/v4_assembly.json` as its resume record; retain that record when resuming the same publication. Use a new build workspace for a different document/revision.

```sh
python src/create_onshape_v4.py --key-file /external/onshape-key.json --document DOCUMENT_ID --workspace WORKSPACE_ID --part-studio PART_STUDIO_ID
python src/onshape_mates.py --key-file /external/onshape-key.json
python src/update_onshape_limits.py --key-file /external/onshape-key.json --apply
python src/onshape_mates.py --key-file /external/onshape-key.json --stage verify
```

The native mate installer resumes by feature name and checks feature status. It creates explicit part-origin-based mate connectors, 317 fastened mates and 18 revolute mates. Each servo case belongs to the upstream rigid group; horns and horn screws belong to the downstream group. The limits and axes match the current joint map.

Keep API credentials in an external mode-0600 JSON with `accessKey` and `secretKey` fields. No credential is needed for local CAD, simulation or training. The committed mate plan contains public document/instance identifiers only. Never reuse its exact instance IDs for a different assembly without regenerating the plan.
