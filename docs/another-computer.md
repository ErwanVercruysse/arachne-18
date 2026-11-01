# Start on another computer

Clone the public repository; all current model assets are ordinary Git files:

```sh
git clone https://github.com/Tom-Michiels/arachne-18.git
cd arachne-18
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r simulation/requirements.txt
python simulation/validate.py
python simulation/grounded_demo.py --headless --seconds 10
```

Windows activation: `.venv\Scripts\activate`. Interactive viewer: `mjpython simulation/grounded_demo.py` on macOS, `python simulation/grounded_demo.py` on Linux/Windows. Use `git pull --ff-only` to retrieve subsequent updates. Onshape credentials and CadQuery are unnecessary for simulation.

The current robot is **mechanical revision v4**. Existing RL checkpoints, results and videos are from earlier geometry. They are preserved, but have not been retrained or performance-validated for v4.

## Prepare training

The CPU MuJoCo/BAM runner works without a GPU. A custom training environment can wrap `simulation/simulate.py`'s `Robot` class: reset with `robot.reset()`, send 18 relative joint-angle targets in `joint_map.json` order, and call `robot.step(target)` once per millisecond. Observations can use joint positions/velocities via the controller indexes, body pose/velocity, IMU readings and contacts. Define rewards, action scaling, terrain and termination in your environment.

For the repository's existing training utilities:

```sh
python -m pip install -r training/requirements.txt
python training/build_model.py
python -m unittest discover -s training -p 'test_*.py'
```

The fast Apple Metal training route additionally requires the external MuJoCo-MLX-Cpp native extension described in [training/README.md](../training/README.md). That route is specific to supported Apple hardware. The CPU BAM model is not automatically an MJX/JAX model; another backend needs its own actuator integration.

Review [joint conventions and model assumptions](simulation.md) before starting new training. No training jobs are started by the setup commands above.
