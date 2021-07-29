from handlers import MovementHandler
from pynput import keyboard

class ActionHandler:
    def __init__(self):
        self.mh = MovementHandler()

    def on_press(self, key):
        if key == keyboard.Key.left:
            self.mh.move_left()
        if key == keyboard.Key.right:
            self.mh.move_right()
        if key == keyboard.Key.up:
            self.mh.move_forward()
        if key == keyboard.Key.down:
            self.mh.move_back()

    def on_release(self, key):
       pass


if __name__ == '__main__':
    ah = ActionHandler()
    with keyboard.Listener(on_press=ah.on_press, on_release=ah.on_release) as listener:
        listener.join()


