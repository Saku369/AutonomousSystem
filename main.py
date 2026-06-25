# -*- coding: utf-8 -*-
import pyglet
from pyglet import *
import random
from material_collecting_environment import MaterialCollectingEnvironment
from material_collecting_agent import MaterialCollectingAgent
from reader_material_collecting_agent import ReaderMaterialCollectingAgent


frame_x, frame_y = 800, 600

window = pyglet.window.Window(frame_x, frame_y)
batch = pyglet.graphics.Batch()

image = pyglet.image.SolidColorImagePattern((100,100,100,255)).create_image(frame_x, frame_y)

f_man = None

auto_executor = True
double_speed_mode = True

class FrameManager:

    def __init__(self) -> None:
        self.environment = None
        self.reset_environment()
        

    def reset_environment(self):
        """
        Reset the environment by deleting all shapes, creating a new MaterialCollectingEnvironment, 
        registering shapes, creating 5 MyMaterialCollectingAgents, and registering them with the environment.
        """
        if self.environment is not None:
            self.environment.delete_shapes()

        mc_env = MaterialCollectingEnvironment(frame_x, frame_y)
        mc_env.register_shapes(batch)

        num_agent = 5
        for i in range(num_agent):
            mc_agent = ReaderMaterialCollectingAgent()
            mc_env.register_agent(mc_agent)

        self.environment = mc_env
    

    def update_frame(self, dt):
        """
        Update the frame using the given time delta.

        :param dt: The time delta for updating the frame.
        :return: None
        """

        self.environment.update(1.0/30.0)

        if auto_executor:
            if self.environment.request_reset:
                self.reset_environment()

@window.event
def on_key_press(symbol, modifiers):
    """
    Event handler for key press events.
    
    Args:
        symbol: The symbol representing the key that was pressed.
        modifiers: Any modifiers applied to the key press event.
        
    Returns:
        None
    """
    if symbol == pyglet.window.key.SPACE:
        f_man.reset_environment()

@window.event
def on_draw():
    """
    Handle the draw event and clear the window, draw an image, and then draw a batch.
    """
    window.clear()
    image.blit(0,0)
    batch.draw()



if __name__ == "__main__":

    f_man = FrameManager()
    
    interval_time = 1.0/30.0
    if double_speed_mode:
        interval_time = 1.0/60.0
    pyglet.clock.schedule_interval(f_man.update_frame,interval_time)
    pyglet.app.run()