#!/usr/bin/env python3

from handlers.video import VideoHandler

class LilyVision(object):

    def __init__(self):
        self.video_handler = VideoHandler()
        self.video_handler.launch_video()

    
if __name__ == '__main__':
    LilyVision()