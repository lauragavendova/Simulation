# Senzory modelu v2 (robot_v2.urdf.xacro) v Isaac Sim, parametre podla
# specifikacii vyrobcov:
#   Hesai XT16               -> /hesai/points  (PointCloud2, hesai_frame)
#   Livox Horizon, len vstavana IMU (BMI088)
#                            -> /livox/imu     (Imu, livox_frame)
#     Samotny Livox lidar sa nesimuluje: solid state lidar s vlastnymi lucmi
#     v Isaac Sim 6.2 nepublikoval ziadne data.
#   Basler a2A1920-160ucPRO  -> /camera/image_raw, /camera/camera_info
#                               (camera_optical_frame)
#
# Polohy senzorov sa citaju z URDF (retazec jointov od frame po base_link),
# senzory su deti prim base_link, takze sa hybu s robotom.
#
# Importuje ho robocup_isaac.py az po starte SimulationApp.
import math
import xml.etree.ElementTree as ET

import omni.graph.core as og
import usdrt.Sdf
from isaacsim.ros2.nodes import (Ros2CameraGraphConfig, Ros2RtxLidarGraphConfig,
                                 create_ros2_camera_graph, create_ros2_rtx_lidar_graph)
from isaacsim.sensors.experimental.physics import IMU
from isaacsim.sensors.experimental.rtx import Lidar
from pxr import Gf, Sdf, UsdGeom, Vt

EMITTER_STATE = 'OmniSensorGenericLidarCoreEmitterStateAPI:s001'
CORE = 'omni:sensor:Core:'
EMITTER = CORE + 'emitterState:s001:'
# nie je v schema OmniLidar, ale profily ho maju (rotary lidar: vystrely za sekundu)
REPORT_RATE = CORE + 'reportRateBaseHz'

# Hesai XT16 (Pandar XT16 user manual): 16 kanalov s krokom 2 deg (+15..-15 deg),
# 360 deg, 10 Hz, horizontalne rozlisenie 0.18 deg, dosah 0.05-120 m, presnost 1 cm,
# 905 nm, single return 320 000 bodov/s
HESAI_ELEVATIONS = [15.0 - 2.0 * i for i in range(16)]
HESAI_HZ = 10
HESAI_AZIMUTH_RES_DEG = 0.18

# Basler a2A1920-160ucPRO: Sony IMX392, 1920 x 1200 px, pixel 3.45 um.
# Objektiv nie je znamy, ohniskova vzdialenost je odhad (6 mm -> ~58 x 38 deg).
CAMERA_SENSOR_PX = (1920, 1200)
# Obraz sa kresli zmenseny: render 1920 x 1200 v kazdom snimku spomali simulaciu
# na ~0.2x realneho casu, 640 x 400 na ~0.6x (RTX 3060 Laptop). Zorne pole je
# rovnake, urcuje ho velkost senzora, nie pocet pixelov obrazu.
CAMERA_RESOLUTION = (640, 400)
CAMERA_PIXEL_MM = 0.00345
CAMERA_FOCAL_MM = 6.0


def frame_pose(urdf, frame):
    """Pose of a URDF link in base_link as Gf.Matrix4d (row-vector convention)."""
    joints = {j.find('child').get('link'): j for j in ET.parse(urdf).getroot().iter('joint')}
    pose = Gf.Matrix4d(1.0)
    link = frame
    while link != 'base_link':
        joint = joints[link]
        origin = joint.find('origin')
        xyz = [float(v) for v in (origin.get('xyz') or '0 0 0').split()]
        r, p, y = [float(v) for v in (origin.get('rpy') or '0 0 0').split()]
        rot = (Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(r))
               * Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(p))
               * Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(y)))
        pose = pose * Gf.Matrix4d().SetTransform(rot, Gf.Vec3d(*xyz))
        link = joint.find('parent').get('link')
    return pose


def set_local_pose(stage, path, pose):
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(pose.ExtractTranslation())
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(pose.ExtractRotationQuat())


def create_lidar(stage, path, pose, hz, attributes):
    # accumulate_outputs: jedna sprava = cela otocka, nie cast za jeden render frame
    Lidar.create(path=path, tick_rate=float(hz), schemas=[EMITTER_STATE], accumulate_outputs=True)
    prim = stage.GetPrimAtPath(path)
    for name, value in attributes.items():
        attr = prim.GetAttribute(name)
        if not attr.IsValid() and name == REPORT_RATE:
            attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.UInt)
        if not attr.IsValid():
            raise RuntimeError(f'{path}: no attribute {name}')
        attr.Set(value)
    set_local_pose(stage, path, pose)


def create_hesai_xt16(stage, path, pose):
    n = len(HESAI_ELEVATIONS)
    # vystrely kanalov rozlozene v ramci jedneho azimutoveho kroku (50 us)
    firing_hz = int(HESAI_HZ * 360.0 / HESAI_AZIMUTH_RES_DEG)
    step_ns = int(1e9 / firing_hz / n)
    create_lidar(stage, path, pose, HESAI_HZ, {
        CORE + 'scanType': 'ROTARY',
        CORE + 'rotationDirection': 'CW',
        CORE + 'scanRateBaseHz': HESAI_HZ,
        CORE + 'patternFiringRateHz': firing_hz,
        REPORT_RATE: firing_hz,
        CORE + 'numberOfEmitters': n,
        CORE + 'numberOfChannels': n,
        CORE + 'nearRangeM': 0.05,
        # lidar je vnutri vlastneho meshu (polomer ~5 cm); luce zacinaju az 10 cm od stredu,
        # inak by ich puzdro zastavilo (nearRangeM len filtruje body, luce neposuva)
        CORE + 'rangeOffsetM': 0.1,
        CORE + 'farRangeM': 120.0,
        CORE + 'minReflectance': 0.1,
        CORE + 'minReflectionRangeM': 80.0,
        CORE + 'rangeAccuracyM': 0.01,
        CORE + 'rangeResolutionM': 0.004,
        CORE + 'maxReturns': 1,
        CORE + 'waveLengthNm': 905.0,
        EMITTER + 'azimuthDeg': Vt.FloatArray([0.0] * n),
        EMITTER + 'elevationDeg': Vt.FloatArray(HESAI_ELEVATIONS),
        EMITTER + 'fireTimeNs': Vt.UIntArray([i * step_ns for i in range(n)]),
        EMITTER + 'channelId': Vt.UIntArray(list(range(1, n + 1))),
    })


def create_basler(stage, path, optical_pose):
    camera = UsdGeom.Camera.Define(stage, path)
    width, height = CAMERA_SENSOR_PX
    # pomer apertura / ohniskova vzdialenost urcuje zorne pole, jednotky sa vykratia
    camera.CreateFocalLengthAttr(CAMERA_FOCAL_MM)
    camera.CreateHorizontalApertureAttr(width * CAMERA_PIXEL_MM)
    camera.CreateVerticalApertureAttr(height * CAMERA_PIXEL_MM)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    # USD kamera sa pozera po -Z s +Y hore, opticky frame ROS po +Z s +Y dole
    flip = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0))
    set_local_pose(stage, path, flip * optical_pose)


def create_imu_graph(imu_path, frame_id, topic):
    keys = og.Controller.Keys
    og.Controller.edit(
        {'graph_path': '/Graph/ROS_LivoxImu', 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('Tick', 'omni.graph.action.OnPlaybackTick'),
                ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
                ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('ReadImu', 'isaacsim.sensors.physics.IsaacReadIMU'),
                ('PubImu', 'isaacsim.ros2.bridge.ROS2PublishImu'),
            ],
            keys.SET_VALUES: [
                ('SimTime.inputs:resetOnStop', True),
                ('ReadImu.inputs:imuPrim', [usdrt.Sdf.Path(imu_path)]),
                ('ReadImu.inputs:readGravity', True),
                ('PubImu.inputs:frameId', frame_id),
                ('PubImu.inputs:topicName', topic),
            ],
            keys.CONNECT: [
                ('Tick.outputs:tick', 'ReadImu.inputs:execIn'),
                ('ReadImu.outputs:execOut', 'PubImu.inputs:execIn'),
                ('ReadImu.outputs:angVel', 'PubImu.inputs:angularVelocity'),
                ('ReadImu.outputs:linAcc', 'PubImu.inputs:linearAcceleration'),
                ('ReadImu.outputs:orientation', 'PubImu.inputs:orientation'),
                ('Context.outputs:context', 'PubImu.inputs:context'),
                ('SimTime.outputs:simulationTime', 'PubImu.inputs:timeStamp'),
            ],
        },
    )


def create_sensors(stage, base_link, urdf):
    """Create the v2 sensors under base_link. Call before setup_simulation."""
    hesai = base_link + '/rtx_hesai'
    camera = base_link + '/sensor_camera'
    imu = base_link + '/sensor_livox_imu'
    create_hesai_xt16(stage, hesai, frame_pose(urdf, 'hesai_frame'))
    create_basler(stage, camera, frame_pose(urdf, 'camera_optical_frame'))
    IMU.create(imu)
    set_local_pose(stage, imu, frame_pose(urdf, 'livox_frame'))
    return {'hesai': hesai, 'camera': camera, 'imu': imu}


def create_sensor_graphs(sensors, camera_resolution=CAMERA_RESOLUTION):
    """ROS 2 publishers for the sensors from create_sensors. Call after setup_simulation."""
    create_ros2_rtx_lidar_graph(Ros2RtxLidarGraphConfig(
        graph_path='/Graph/ROS_Hesai', lidar_prim=sensors['hesai'],
        frame_id='hesai_frame', publish_laser_scan=False,
        publish_point_cloud=True, point_cloud_topic='/hesai/points'))
    graph = create_ros2_camera_graph(Ros2CameraGraphConfig(
        graph_path='/Graph/ROS_Camera', camera_prim=sensors['camera'],
        frame_id='camera_optical_frame', camera_info_topic='/camera/camera_info',
        rgb_topic='/camera/image_raw', publish_depth=False))
    width, height = camera_resolution
    og.Controller.attribute(graph + '/RenderProduct.inputs:width').set(width)
    og.Controller.attribute(graph + '/RenderProduct.inputs:height').set(height)
    create_imu_graph(sensors['imu'], 'livox_frame', '/livox/imu')
