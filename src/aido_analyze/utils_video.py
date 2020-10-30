import sys

from duckietown_world.svg_drawing.draw_log import SimulatorLog
from procgraph import Block, Generator, pg, register_model_spec

from .utils_drawing import log_summary, read_simulator_log_cbor, read_topic2


class CBORRead(Generator):
    """
    """

    Block.alias("cborread")
    Block.output("image")
    Block.config("filename", "CBOR file to read")
    Block.config("robot_name", "robot name")

    # noinspection PyAttributeOutsideInit
    def init(self):
        fn = self.get_config("filename")
        robot_name = self.get_config("robot_name")
        self.ld = log_summary(fn)
        self.log: SimulatorLog = read_simulator_log_cbor(self.ld)
        self.i = 0
        if robot_name not in self.log.robots:
            msg = f"Cannot find robot {robot_name} among {list(self.log.robots)}"
            raise Exception(msg)

        log = self.log.robots[robot_name]
        self.n = len(log.observations)
        self.robot_log = log

    def next_data_status(self):

        if self.i < self.n:
            return True, self.robot_log.observations.timestamps[self.i]
        else:
            return False, None

    def update(self):
        i = self.i
        timestamp = self.robot_log.observations.timestamps[i]
        value = self.robot_log.observations.values[i]
        self.set_output("image", value=value, timestamp=timestamp)

        self.i += 1

    def finish(self):
        pass


class CBORReadTopic(Generator):
    """
    """

    Block.alias("cborread_topic")
    Block.output("image")
    Block.config("filename", "CBOR file to read")
    Block.config("topic", "which topic to display")

    # noinspection PyAttributeOutsideInit
    def init(self):
        fn = self.get_config("filename")
        topic = self.get_config("topic")
        self.ld = log_summary(fn)
        self.topics = list(read_topic2(self.ld, topic))
        # print(self.topics)
        self.i = 0
        self.n = len(self.topics)

    def next_data_status(self):
        if self.i < self.n:
            return True, None
        else:
            return False, None

    def update(self):
        i = self.i
        timestamp = i * 0.04  # XXX
        jpg_image = self.topics[i]["data"]
        # print(f'jpg_image: {jpg_image}')
        value = jpg_image["jpg_data"]
        self.set_output("image", value=value, timestamp=timestamp)

        self.i += 1

    def finish(self):
        pass


def make_video1(*, log_filename: str, robot_name: str, output_video: str) -> None:
    register_model_spec(
        """
    --- model video_aido
    config output
    config filename
    config robot_name


    |cborread filename=$filename robot_name=$robot_name| --> |jpg2rgb| -> rgb
    rgb -> |identity| -> retimed
    # rgb --> |rewrite_timestamps interval=$factor| -> retimed
    retimed --> |info|
    retimed -> |mencoder quiet=1 file=$output timestamps=0|

        """
    )

    pg(
        "video_aido", dict(filename=log_filename, output=output_video, robot_name=robot_name),
    )


def aido_log_video_main():
    make_video1(
        log_filename=sys.argv[1], output_video="out-aido-log-video.mp4", robot_name=sys.argv[2],
    )


def make_video_ui_image(*, log_filename: str, output_video: str) -> None:
    register_model_spec(
        """
    --- model video_aido_ui_image
    config output
    config filename
    config topic


    |cborread_topic filename=$filename topic=$topic| --> |jpg2rgb| -> rgb
    rgb -> |identity| -> retimed
    # rgb --> |rewrite_timestamps interval=$factor| -> retimed
    retimed --> |info|
    retimed -> |mencoder quiet=1 file=$output timestamps=0 firstpass_bitrate=120000000|

        """
    )

    params = dict(filename=log_filename, output=output_video, topic="ui_image")
    pg("video_aido_ui_image", params)


def aido_log_video_ui_image_main():
    make_video_ui_image(
        log_filename=sys.argv[1], output_video="out-aido-log-video_ui_image.mp4",
    )


if __name__ == "__main__":
    aido_log_video_main()
