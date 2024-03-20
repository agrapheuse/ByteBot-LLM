import subprocess
from typing import Dict, Optional, Tuple, Union

import rclpy
from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from langchain.tools import BaseTool

from ._dance_helper import *


class DanceTool(BaseTool):
    name = "dance"
    description = "Useful for dancing. Use this to dance around the robot. Express yourself! Doesn't require any parameters"

    def __init__(self):
        super().__init__()

    def _run(
        self,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ):
        """
        Publishes a string message to a topic.
        """
        cp = ColorPalette()
        """
        DANCE SEQUENCE is defined as a list of pairs with 
        (time to start action in seconds, action to take)
        """
        DANCE_SEQUENCE = [
            (
                0.0,
                Lights([cp.green, cp.green, cp.green, cp.green, cp.green, cp.green]),
            ),
            (0.0, Move(0.0, 0.0)),
            (0.5, Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
            (0.5, Move(0.15, 70)),
            (1.0, Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
            (1.0, Move(0.15, -70)),
            (1.5, Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
            (1.5, Move(0.15, 70)),
            (2.0, Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
            (2.0, Move(0.15, -70)),
            (2.5, Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
            (2.5, Move(-0.15, 70)),
            (3.0, Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
            (3.0, Move(-0.15, -70)),
            (3.5, Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
            (3.5, Move(-0.15, 70)),
            (4.0, Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
            (4.0, Move(-0.15, -70)),
            (4.5, Lights([cp.red, cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan])),
            (4.5, Move(0.0, 60)),
            (5.0, Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
            (5.5, Lights([cp.pink, cp.cyan, cp.red, cp.green, cp.blue, cp.yellow])),
            (6.0, Lights([cp.yellow, cp.pink, cp.cyan, cp.red, cp.green, cp.blue])),
            (6.5, Lights([cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red, cp.green])),
            (6.5, Move(0.0, -75)),
            (7.0, Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
            (7.5, Lights([cp.pink, cp.cyan, cp.red, cp.green, cp.blue, cp.yellow])),
            (8.0, Lights([cp.yellow, cp.pink, cp.cyan, cp.red, cp.green, cp.blue])),
            (8.5, Lights([cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red, cp.green])),
            (9.0, Lights([cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red])),
            (9.5, Lights([cp.red, cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan])),
            (9.5, Move(-0.15, 50)),
            (10.0, Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
            (10.0, Move(-0.15, -50)),
            (10.5, Lights([cp.pink, cp.cyan, cp.red, cp.green, cp.blue, cp.yellow])),
            (10.5, Move(-0.15, 50)),
            (11.0, Lights([cp.yellow, cp.pink, cp.cyan, cp.red, cp.green, cp.blue])),
            (11.0, Move(-0.15, -50)),
            (11.5, Lights([cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red, cp.green])),
            (11.5, Move(0.0, 75)),
            (13.0, Lights([cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red])),
            (13.0, Move(0.0, 75)),
            (14.0, Lights([cp.red, cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan])),
            (14.0, Move(0.0, -100)),
            (15.5, Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
            (15.5, Move(0.0, 100)),
            (
                20.5,
                Lights([cp.red, cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow]),
            ),
            (20.5, Move(-0.15, -70)),
            (
                21.0,
                Lights([cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow, cp.red]),
            ),
            (21.0, Move(-0.15, 70)),
            (
                21.5,
                Lights([cp.red, cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow]),
            ),
            (21.5, Move(0.15, -70)),
            (
                22.0,
                Lights([cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow, cp.red]),
            ),
            (22.0, Move(0.15, 70)),
            (22.5, FinishedDance()),
        ]

        dance_publisher = None
        try:
            dance_choreographer = DanceChoreographer(DANCE_SEQUENCE)
            dance_publisher = DanceCommandPublisher(dance_choreographer)
            song_link = self._get_random_song_link()
            subprocess.Popen(
                [
                    "mpv",
                    "--audio-device=alsa/hw:1,0",
                    song_link,
                    "--no-video",
                ]
            )
            rclpy.spin(dance_publisher)
        except FinishedDance:
            # kill above subprocess
            subprocess.run(["pkill", "mpv"])
            print("Finished Dance")

    async def _arun(
        self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        pass

    def _to_args_and_kwargs(self, tool_input: Union[str, Dict]) -> Tuple[Tuple, Dict]:
        return (), {}

    def _get_random_song_link(self):
        import random

        songs = [
            "https://www.youtube.com/watch?v=sFZjqVnWBhc",  # Robot Rock
            "https://www.youtube.com/watch?v=eSBybJGZoCU",  # Pocket Calculator
            # "https://youtu.be/7YGXDYWCyoY?si=h16kWwxKe-D8Xcu_&t=171",  # Synthetic God
            # "https://www.youtube.com/watch?v=aoBniaBMLGk",  # Future Behold
        ]
        # Choose a random song
        index = random.randint(0, len(songs) - 1)
        return songs[0]
