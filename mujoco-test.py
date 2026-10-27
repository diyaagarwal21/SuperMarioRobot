import time
import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('STLs/scene.xml')
d = mujoco.MjData(m)

# Spawn at the "home" keyframe (standing, feet 1 mm above the floor)
mujoco.mj_resetDataKeyframe(m, d, 0)
mujoco.mj_forward(m, d)


def main():
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(m, d)
            viewer.sync()

            # Keep the sim roughly real-time
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()