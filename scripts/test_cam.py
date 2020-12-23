import threading
import anki_vector
from anki_vector import events

def on_new_raw_camera_image(robot, event_type, event, done):
    print("Display new camera image")
    event.image.show()
    done.set()

with anki_vector.Robot() as robot:
    robot.camera.init_camera_feed()
    done = threading.Event()
    robot.events.subscribe(on_new_raw_camera_image, events.Events.new_raw_camera_image, done)

    print("------ waiting for camera events, press ctrl+c to exit early ------")

    try:
        if not done.wait(timeout=5):
            print("------ Did not receive a new camera image! ------")
    except KeyboardInterrupt:
        pass
