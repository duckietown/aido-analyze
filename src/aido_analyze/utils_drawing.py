import sys
from collections import defaultdict
from dataclasses import dataclass
from typing import cast, Dict, Iterator, List, Optional

import cbor2
import geometry
import geometry as g
import numpy as np
import yaml
from cbor2 import CBORDecodeEOF
from zuper_ipce import IEDO, object_from_ipce
from zuper_ipce.json2cbor import tag_hook

from aido_schemas.protocol_simulator import RobotObservations, RobotState, SetRobotCommands
from aido_schemas.schemas import DB20Observations, DTSimRobotInfo
from duckietown_world import draw_static, DuckietownMap, SampledSequence, SE2Transform
from duckietown_world.rules import evaluate_rules
from duckietown_world.rules.rule import make_timeseries, RuleEvaluationResult
from duckietown_world.seqs.tsequence import SampledSequenceBuilder
from duckietown_world.svg_drawing.draw_log import RobotTrajectories, SimulatorLog
from duckietown_world.svg_drawing.misc import TimeseriesPlot
from duckietown_world.world_duckietown import construct_map, DB18
from duckietown_world.world_duckietown.utils import get_velocities_from_sequence
from . import logger


@dataclass
class LogData:
    objects: List[dict]


def log_summary(filename: str) -> LogData:
    objects = []
    with open(filename, "rb") as f:
        counts = defaultdict(lambda: 0)
        sizes = defaultdict(lambda: 0)

        while True:
            try:
                ob = cbor2.load(f, tag_hook=tag_hook)
            except CBORDecodeEOF:
                break

            # for ob in read_cbor_or_json_objects(f):
            topic = ob["topic"]
            size_ob = 0
            # size_ob = len(cbor2.dumps(ob))
            counts[topic] += 1
            sizes[topic] += size_ob
            objects.append(ob)

    ordered = sorted(counts, key=lambda x: sizes[x], reverse=True)
    for topic in ordered:
        count = counts[topic]
        size_mb = sizes[topic] / (1024 * 1024.0)
        logger.info("topic %25s: %4s messages  %.2f MB" % (topic, count, size_mb))
    return LogData(objects)


def read_topic2(ld: LogData, topic: str) -> Iterator[dict]:
    for ob in ld.objects:
        if ob["topic"] == topic:
            yield ob


def read_map_info(ld: LogData) -> DuckietownMap:
    m = list(read_topic2(ld, "set_map"))
    if not m:
        msg = "Could not find set_map"
        raise Exception(msg)
    m = m[0]
    map_data_yaml = m["data"]["map_data"]
    map_data = yaml.load(map_data_yaml, Loader=yaml.SafeLoader)
    duckietown_map = construct_map(map_data)
    return duckietown_map


def read_perfomance(ld: LogData) -> Dict[str, RuleEvaluationResult]:
    sb = SampledSequenceBuilder[float]()
    sb.add(0, 0)
    sequences: Dict[str, SampledSequenceBuilder] = defaultdict(lambda: SampledSequenceBuilder[float]())

    for i, ob in enumerate(read_topic2(ld, "timing_information")):
        # ob = ipce_to_object(ob['data'], {}, {})
        phases = ob["data"]["phases"]
        phases.pop("$schema")
        for p, f in phases.items():
            sequences[p].add(t=i, v=f)

    evr = RuleEvaluationResult(None)
    for p, sb in sequences.items():
        seq = sb.as_sequence()
        total = float(np.mean(seq.values))
        evr.set_metric((p,), total=total, incremental=seq, cumulative=None)

    return {"performance": evr}


def read_trajectories(ld: LogData) -> Dict[str, RobotTrajectories]:
    rs = list(read_topic2(ld, "robot_state"))
    if not rs:
        msg = "Could not find robot_state"
        raise Exception(msg)
    robot_names = set([r["data"]["robot_name"] for r in rs])
    logger.info(f"robot_names: {robot_names}")

    robot2trajs = {}
    for robot_name in robot_names:
        ssb_pose = SampledSequenceBuilder[SE2Transform]()
        ssb_pose_SE2 = SampledSequenceBuilder[g.SE2value]()
        # ssb_actions = SampledSequenceBuilder[object]()
        # ssb_wheels_velocities = SampledSequenceBuilder[object]()
        # ssb_velocities = SampledSequenceBuilder[Any]()
        for r in rs:
            found = object_from_ipce(r["data"], iedo=iedo)
            robot_state = cast(RobotState, found)
            if robot_state.robot_name != robot_name:
                continue
            state = cast(DTSimRobotInfo, robot_state.state)
            pose = state.pose
            # velocity = robot_state.state.velocity
            # last_action = state.last_action
            # wheels_velocities = state.wheels_velocities

            t = robot_state.t_effective
            ssb_pose_SE2.add(t, pose)
            ssb_pose.add(t, SE2Transform.from_SE2(pose))
            # ssb_actions.add(t, last_action)
            # ssb_wheels_velocities.add(t, wheels_velocities)
            # ssb_velocities.add(t, velocity)

        seq_velocities = get_velocities_from_sequence(ssb_pose_SE2)
        observations = read_observations(ld, robot_name)
        commands = read_commands(ld, robot_name)

        robot2trajs[robot_name] = RobotTrajectories(
            pose=ssb_pose.as_sequence(),
            # ssb_actions.as_sequence(),
            # ssb_wheels_velocities.as_sequence(),
            velocity=seq_velocities,
            observations=observations,
            commands=commands,
        )
    return robot2trajs


def read_observations(ld: LogData, robot_name: str) -> SampledSequence:
    ssb = SampledSequenceBuilder[bytes]()
    obs = list(read_topic2(ld, "robot_observations"))
    last_t = None
    for ob in obs:
        found = object_from_ipce(ob["data"], iedo=iedo)

        ro = cast(RobotObservations, found)
        if ro.robot_name != robot_name:
            continue
        do = cast(DB20Observations, ro.observations)

        t = ro.t_effective
        camera = do.camera.jpg_data

        if last_t != t:
            ssb.add(t, camera)
        last_t = t
    res = ssb.as_sequence()
    return res


iedo = IEDO(use_remembered_classes=True, remember_deserialized_classes=True)


def read_commands(ld: LogData, robot_name: str) -> SampledSequence:
    ssb = SampledSequenceBuilder[SetRobotCommands]()
    obs = list(read_topic2(ld, "set_robot_commands"))
    last_t = None
    for ob in obs:
        found = object_from_ipce(ob["data"], iedo=iedo)
        ro = cast(SetRobotCommands, found)
        if ro.robot_name != robot_name:
            continue
        t = ro.t_effective
        if last_t != t:
            ssb.add(ro.t_effective, ro.commands)
        last_t = t
    seq = ssb.as_sequence()
    if len(seq) == 0:
        msg = f'Could not find any robot_commands in the log for robot "{robot_name}".'
        logger.warning(msg)
    return seq


def read_simulator_log_cbor(ld: LogData, main_robot_name: Optional[str] = None) -> SimulatorLog:
    render_time = read_perfomance(ld)
    duckietown_map = read_map_info(ld)
    robots = read_trajectories(ld)
    # logger.info(f'robots: {len(robots)}')

    for robot_name, trajs in robots.items():
        # logger.info(f'robots: {robot_name} trajs: {trajs.pose.get_sampling_points()}')
        if robot_name == main_robot_name:
            color = "red"
        elif "ego" in robot_name:
            color = "pink"
        elif "parked" in robot_name:
            color = "blue"
        elif "npc" in robot_name:
            color = "yellow"
        else:
            color = "grey"

        robot = DB18(color=color)
        # noinspection PyTypeChecker
        duckietown_map.set_object(robot_name, robot, ground_truth=trajs.pose)

    return SimulatorLog(duckietown=duckietown_map, robots=robots, render_time=render_time)


def evaluate_stats(fn: str, robot_main: str) -> Dict[str, RuleEvaluationResult]:
    ld = log_summary(fn)
    log0 = read_simulator_log_cbor(ld, main_robot_name=robot_main)
    log = log0.robots[robot_main]
    duckietown_env = log0.duckietown
    interval = SampledSequence.from_iterator(enumerate(log.pose.timestamps))
    evaluated = evaluate_rules(
        poses_sequence=log.pose, interval=interval, world=duckietown_env, ego_name=robot_main,
    )
    return evaluated


def read_and_draw(fn: str, output: str, robot_main: str) -> Dict[str, RuleEvaluationResult]:
    ld = log_summary(fn)

    logger.info("Reading logs...")
    log0 = read_simulator_log_cbor(ld, main_robot_name=robot_main)
    logger.info("...done")

    # for robot_main in pc_names:
    if not robot_main in log0.robots:
        msg = f"Cannot find robot {robot_main!r}"
        raise Exception(msg)

    log = log0.robots[robot_main]

    if log.observations:
        images = {"observations": log.observations}
    else:
        images = None
    duckietown_env = log0.duckietown
    timeseries = {}

    # logger.info("Computing timeseries_actions...")
    # timeseries.update(timeseries_actions(log))
    logger.info("Computing timeseries_wheels_velocities...")
    timeseries.update(timeseries_wheels_velocities(log.commands))
    logger.info("Computing timeseries_robot_velocity...")
    timeseries.update(timeseries_robot_velocity(log.velocity))
    logger.info("Evaluating rules...")
    interval = SampledSequence.from_iterator(enumerate(log.pose.timestamps))
    evaluated = evaluate_rules(
        poses_sequence=log.pose, interval=interval, world=duckietown_env, ego_name=robot_main,
    )
    if True:
        evaluated.update(log0.render_time)

    for k, v in evaluated.items():
        for kk, vv in v.metrics.items():
            logger.info("%20s %20s %s" % (k, kk, vv))
    timeseries.update(make_timeseries(evaluated))
    logger.info("Drawing...")
    draw_static(
        duckietown_env, output, images=images, timeseries=timeseries, main_robot_name=robot_main,
    )
    logger.info("...done.")
    return evaluated


def timeseries_wheels_velocities(log_commands: SampledSequence,) -> Dict[str, TimeseriesPlot]:
    timeseries = {}
    sequences = {}

    def get_left(_):
        return _.wheels.motor_left

    def get_right(_):
        return _.wheels.motor_right

    sequences["motor_left"] = log_commands.transform_values(get_left, float)
    sequences["motor_right"] = log_commands.transform_values(get_right, float)
    timeseries["pwm_commands"] = TimeseriesPlot(
        "PWM commands", "The PWM commands sent to the wheels", sequences
    )
    return timeseries


def timeseries_robot_velocity(log_velocity: SampledSequence,) -> Dict[str, TimeseriesPlot]:
    timeseries = {}
    sequences = {}

    # logger.info(log_velocity)

    def speed(x) -> float:
        l, omega = geometry.linear_angular_from_se2(x)
        return l[0]

    def omega(x) -> float:
        l, omega = geometry.linear_angular_from_se2(x)
        return omega

    sequences["linear_speed"] = log_velocity.transform_values(lambda _: speed(_), float)
    sequences["angular_velocity"] = log_velocity.transform_values(lambda _: omega(_), float)
    # logger.info("linear speed: %s" % sequences["linear_speed"])
    # logger.info("angular velocity: %s" % sequences["angular_velocity"])
    timeseries["velocity"] = TimeseriesPlot("Velocities", "velocities", sequences)
    return timeseries


def aido_log_draw_main():
    read_and_draw(sys.argv[1], "test", robot_main=sys.argv[2])


if __name__ == "__main__":
    # in case called
    read_and_draw(sys.argv[2], "test", robot_main=sys.argv[3])
