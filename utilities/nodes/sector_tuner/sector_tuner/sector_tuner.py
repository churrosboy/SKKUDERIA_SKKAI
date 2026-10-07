import rclpy
from copy import deepcopy
from rcl_interfaces.msg import ParameterType, ParameterDescriptor, FloatingPointRange
from rclpy.node import Node
from f110_msgs.msg import WpntArray
import numpy as np
import trajectory_planning_helpers as tph
from visualization_msgs.msg import MarkerArray, Marker
from tf_transformations import quaternion_from_euler

from frenet_conversion.frenet_converter import FrenetConverter
from stack_master.parameter_event_handler import ParameterEventHandler

LANE_SIGNS = {'left': 1.0, 'middle': 0.0, 'right': -1.0}
LANE_NONE_VALUES = (None, '', 'none', 'raceline')


def build_lane_d_targets(sectors_params, n_sectors, d_left, d_right, n_unique,
                         blend_idx=30, margin=0.15, logger=None):
    """
    Per-waypoint lateral target d (raceline frenet, +left) for the lane sectors in
    sectors_params. Returns an array of length n_unique (the duplicated closing
    waypoint is excluded); all-zero when no sector sets a lane.
    Piecewise targets are clamped inside the track bounds (margin to the wall) and
    blended circularly with a moving average -> linear ramps of ~blend_idx indices
    (0.1 m spacing) on each side of a sector boundary.
    """
    d_tgt = np.zeros(n_unique)
    any_lane = False
    window = 2 * blend_idx + 1
    for i in range(n_sectors):
        sec = sectors_params.get(f'Sector{i}', {})
        lane = sec.get('lane', 'raceline')
        if lane in LANE_NONE_VALUES:
            continue
        if lane not in LANE_SIGNS:
            if logger is not None:
                logger.warn(f"[Sector Tuner] Sector{i}: unknown lane '{lane}' "
                            f"(expected one of {list(LANE_SIGNS)}), ignoring")
            continue
        offset = float(sec.get('lane_offset', 0.3))
        start = int(sec['start'])
        end = min(int(sec['end']), n_unique - 1)
        val = np.clip((d_left - d_right) / 2.0 + LANE_SIGNS[lane] * offset,
                      -(d_right - margin), d_left - margin)
        mask = np.zeros(n_unique)
        mask[start:end + 1] = 1.0
        padded = np.pad(mask, blend_idx, mode='wrap')
        w = np.convolve(padded, np.ones(window) / window, mode='same')[blend_idx:-blend_idx]
        d_tgt += w * val
        any_lane = True
        if logger is not None and (end - start) < 2 * blend_idx:
            logger.warn(f"[Sector Tuner] Sector{i} lane '{lane}': sector length "
                        f"{end - start} idx < 2*blend ({2 * blend_idx}) -> full lane "
                        "offset is never reached (ramps overlap)")
    if not any_lane:
        return d_tgt
    d_tgt = np.clip(d_tgt, -(d_right - margin), d_left - margin)
    return d_tgt


def apply_lane_geometry(wpnts_og: WpntArray, d_tgt: np.ndarray, converter: FrenetConverter) -> WpntArray:
    """
    Returns a deepcopy of wpnts_og with the lane offsets d_tgt applied: x/y moved via
    the FrenetConverter, psi/kappa recomputed numerically on the offset line (pp.py's
    L1 curvature cap reads local-waypoint kappa), d_m set to the offset and
    d_left/d_right adjusted so bound distances stay valid for the moved points.
    s_m / vx_mps / ax_mps2 are kept from the raceline (velocity stays raceline profile
    x sector scaling). Untouched (|d| < 1e-4) waypoints stay byte-identical.
    """
    out = deepcopy(wpnts_og)
    if len(d_tgt) == 0 or np.max(np.abs(d_tgt)) < 1e-4:
        return out
    n_unique = len(d_tgt)
    s_arr = np.array([w.s_m for w in wpnts_og.wpnts[:n_unique]])
    xy = converter.get_cartesian(s_arr, d_tgt)
    x_new = np.asarray(xy[0], dtype=float)
    y_new = np.asarray(xy[1], dtype=float)
    seg_lengths = np.hypot(np.diff(np.r_[x_new, x_new[0]]),
                           np.diff(np.r_[y_new, y_new[0]]))
    psi_new, kappa_new = tph.calc_head_curv_num.calc_head_curv_num(
        path=np.column_stack((x_new, y_new)),
        el_lengths=seg_lengths,
        is_closed=True,
    )
    psi_new += np.pi / 2

    for i in range(n_unique):
        if abs(d_tgt[i]) < 1e-4:
            continue
        wpnt = out.wpnts[i]
        wpnt.x_m = float(x_new[i])
        wpnt.y_m = float(y_new[i])
        wpnt.psi_rad = float(psi_new[i])
        wpnt.kappa_radpm = float(kappa_new[i])
        wpnt.d_m = float(d_tgt[i])
        wpnt.d_left = wpnts_og.wpnts[i].d_left - float(d_tgt[i])
        wpnt.d_right = wpnts_og.wpnts[i].d_right + float(d_tgt[i])
    if abs(d_tgt[0]) >= 1e-4 and len(out.wpnts) == n_unique + 1:
        first, last = out.wpnts[0], out.wpnts[-1]
        last.x_m = first.x_m
        last.y_m = first.y_m
        last.psi_rad = first.psi_rad
        last.kappa_radpm = first.kappa_radpm
        last.d_m = first.d_m
        last.d_left = first.d_left
        last.d_right = first.d_right
    return out


class SectorTuner(Node):
    """
    Sector scaler for the velocity of the global waypoints
    """
    def __init__(self):
        super().__init__('sector_tuner',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vis_timer = self.create_timer(1.0, self.marker_callback)
        
        # sectors params
        self.glb_wpnts_og = None
        self.glb_wpnts_scaled = None
        self.glb_wpnts_sp_og = None
        self.glb_wpnts_sp_scaled = None
        self.glb_wpnts_lane_base = None
        self.converter = None

        # get initial scaling
        self.sectors_params=self.parameters_to_dict()
        self.n_sectors = self.sectors_params['n_sectors']
        for i in range(self.n_sectors):
            self.sectors_params[f"Sector{i}"]['scaling'] = np.clip(
                self.sectors_params[f"Sector{i}"]['scaling'], 0, self.sectors_params['global_limit']
            )
        
        desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.01)])
        self.set_descriptor('global_limit',descriptor=desc)
        for i in range(self.n_sectors):
            self.set_descriptor('Sector'+str(i)+'.scaling',descriptor=desc)

        # dyn params sub
        self.glb_wpnts_name = "/global_waypoints"
        self.handler = ParameterEventHandler(self)
        self.callback_handle = self.handler.add_parameter_event_callback(
            callback=self.dyn_param_cb,
        )
        self.global_waypoint_sub = self.create_subscription(
            WpntArray,
            self.glb_wpnts_name,
            self.global_waypoints_cb,
            10)
        self.global_waypoint_sp_sub = self.create_subscription(
            WpntArray,
            self.glb_wpnts_name+"/shortest_path",
            self.global_waypoints_sp_cb,
            10)
        
        # new glb_waypoints pub
        self.scaled_points_pub = self.create_publisher(WpntArray, "/global_waypoints_scaled", 10)
        self.scaled_points_sp_pub = self.create_publisher(WpntArray, "/global_waypoints_scaled/shortest_path", 10)
        
        # Visualizations
        self.sector_visualization_pub = self.create_publisher(MarkerArray, '/sector_markers', 10)
        
        self.get_logger().info("Waiting for global waypoints...")
        
    def parameters_to_dict(self):
        params = {}
        for key in self._parameters:
            keylist = key.split('.')
            paramit = params
            for subkey in keylist[:-1]:
                paramit = paramit.setdefault(subkey, {})
            paramit[keylist[-1]] = self.get_parameter(key).value
        return params

    def global_waypoints_cb(self, data:WpntArray):
        """
        Saves the global waypoints of the main trajectory (e.g. min curvature)
        """
        self.glb_wpnts_og = data
        self.glb_wpnts_lane_base = None
        self.converter = None
     
    def global_waypoints_sp_cb(self, data:WpntArray):
        """
        Saves the global waypoints of the shortest path
        """
        self.glb_wpnts_sp_og = data

    def dyn_param_cb(self, parameter_event):
        """
        Notices the change in the parameters and scales the global waypoints
        """
        if(parameter_event.node != '/sector_tuner'):
            return
        self.sectors_params = self.parameters_to_dict()
        # update params 
        for i in range(self.n_sectors):
            self.sectors_params[f"Sector{i}"]['scaling'] = np.clip(
                self.sectors_params[f"Sector{i}"]['scaling'], 0, self.sectors_params['global_limit']
            )
        self.glb_wpnts_lane_base = None

        self.get_logger().info(str(self.sectors_params))

    def get_vel_scaling(self, s):
        """
        Gets the dynamically reconfigured velocity scaling for the points.
        Linearly interpolates for points between two sectors
        
        Parameters
        ----------
        s
            s parameter whose sector we want to find
        """
        hl_change = 15

        if self.n_sectors > 1:
            for i in range(self.n_sectors):
                if i == 0 :
                    if (s >= self.sectors_params[f'Sector{i}']['start']) and (s < self.sectors_params[f'Sector{i}']['start'] + hl_change):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i}']['start']-hl_change, self.sectors_params[f'Sector{i}']['start']+hl_change],
                            fp=[self.sectors_params[f'Sector{self.n_sectors-1}']['scaling'], self.sectors_params[f'Sector{i}']['scaling']]
                        )
                    elif (s >= self.sectors_params[f'Sector{i}']['start'] + hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start'] - hl_change):
                        scaler = self.sectors_params[f"Sector{i}"]['scaling']
                    elif (s >= self.sectors_params[f'Sector{i+1}']['start'] - hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start']):
                        scaler = np.interp(
                        x=s,
                        xp=[self.sectors_params[f'Sector{i+1}']['start']-hl_change, self.sectors_params[f'Sector{i+1}']['start']+hl_change],
                        fp=[self.sectors_params[f'Sector{i}']['scaling'], self.sectors_params[f'Sector{i+1}']['scaling']]
                    )
                elif i != self.n_sectors-1:
                    if (s >= self.sectors_params[f'Sector{i}']['start']) and (s < self.sectors_params[f'Sector{i}']['start'] + hl_change):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i}']['start']-hl_change, self.sectors_params[f'Sector{i}']['start']+hl_change],
                            fp=[self.sectors_params[f'Sector{i-1}']['scaling'], self.sectors_params[f'Sector{i}']['scaling']]
                        )
                    elif (s >= self.sectors_params[f'Sector{i}']['start'] + hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start'] - hl_change):
                        scaler = self.sectors_params[f"Sector{i}"]['scaling']
                    elif (s >= self.sectors_params[f'Sector{i+1}']['start'] - hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start']):
                        scaler = np.interp(
                        x=s,
                        xp=[self.sectors_params[f'Sector{i+1}']['start']-hl_change, self.sectors_params[f'Sector{i+1}']['start']+hl_change],
                        fp=[self.sectors_params[f'Sector{i}']['scaling'], self.sectors_params[f'Sector{i+1}']['scaling']]
                    )
                else:
                    if (s >= self.sectors_params[f'Sector{i}']['start']) and (s < self.sectors_params[f'Sector{i}']['start'] + hl_change):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i}']['start']-hl_change, self.sectors_params[f'Sector{i}']['start']+hl_change],
                            fp=[self.sectors_params[f'Sector{i-1}']['scaling'], self.sectors_params[f'Sector{i}']['scaling']]
                        )
                    elif (s >= self.sectors_params[f'Sector{i}']['start'] + hl_change) and (s < self.sectors_params[f'Sector{i}']['end'] - hl_change):
                        scaler = self.sectors_params[f"Sector{i}"]['scaling']
                    elif (s >= self.sectors_params[f'Sector{i}']['end'] - hl_change):
                        scaler = np.interp(
                        x=s,
                        xp=[self.sectors_params[f'Sector{i}']['end']-hl_change, self.sectors_params[f'Sector{i}']['end']+hl_change],
                        fp=[self.sectors_params[f'Sector{i}']['scaling'], self.sectors_params[f'Sector{0}']['scaling']]
                    )
        elif self.n_sectors == 1:
            scaler = self.sectors_params["Sector0"]['scaling']

        return scaler

    def compute_lane_base(self) -> WpntArray:
        """
        Builds the geometry base for the scaled waypoints: the raceline with any
        per-sector lane offsets (SectorN.lane in speed_scaling.yaml) spliced in.
        Returns a plain deepcopy of the raceline when no lane is configured.
        """
        wpnts = self.glb_wpnts_og.wpnts
        n_unique = len(wpnts) - 1
        if self.converter is None:
            self.converter = FrenetConverter(
                np.array([w.x_m for w in wpnts]),
                np.array([w.y_m for w in wpnts]),
                np.array([w.psi_rad for w in wpnts]),
            )
        d_left = np.array([w.d_left for w in wpnts[:n_unique]])
        d_right = np.array([w.d_right for w in wpnts[:n_unique]])
        blend_idx = int(self.sectors_params.get('lane_blend_idx', 30))
        d_tgt = build_lane_d_targets(
            self.sectors_params, self.n_sectors, d_left, d_right, n_unique,
            blend_idx=blend_idx, logger=self.get_logger(),
        )
        if np.max(np.abs(d_tgt)) >= 1e-4:
            self.get_logger().info(
                f"[Sector Tuner] lane override active, max |d| = {np.max(np.abs(d_tgt)):.2f} m")
        return apply_lane_geometry(self.glb_wpnts_og, d_tgt, self.converter)

    def scale_points(self):
        """
        Scales the global waypoints' velocities
        """
        if self.glb_wpnts_lane_base is None:
            self.glb_wpnts_lane_base = self.compute_lane_base()
            self.glb_wpnts_scaled = None
        if self.glb_wpnts_scaled is None:
            self.glb_wpnts_scaled = deepcopy(self.glb_wpnts_lane_base)
            self.glb_wpnts_sp_scaled = deepcopy(self.glb_wpnts_sp_og)

        for i, wpnt  in enumerate(self.glb_wpnts_og.wpnts):
            vel_scaling = self.get_vel_scaling(i)
            new_vel = wpnt.vx_mps*vel_scaling
            self.glb_wpnts_scaled.wpnts[i].vx_mps = new_vel

    def timer_callback(self):
        if(self.glb_wpnts_og is None):
            return
        self.scale_points()
        self.scaled_points_pub.publish(self.glb_wpnts_scaled)
        
    def marker_callback(self):
        if self.glb_wpnts_og is None:
            return
        if self.sector_visualization_pub.get_subscription_count() == 0:
            return
        
        global_waypoints_vis = []
        for waypoint in self.glb_wpnts_og.wpnts:
            global_waypoints_vis.append([waypoint.x_m, waypoint.y_m, waypoint.s_m])
        
        n_sectors = self.sectors_params['n_sectors']
        sec_markers = MarkerArray()

        for i in range(n_sectors):
            s = self.sectors_params[f"Sector{i}"]['start']
            if s == (len(global_waypoints_vis) - 1):
                theta = np.arctan2((global_waypoints_vis[0][1] - global_waypoints_vis[s][1]),(global_waypoints_vis[0][0] - global_waypoints_vis[s][0]))
            else:
                theta = np.arctan2((global_waypoints_vis[s+1][1] - global_waypoints_vis[s][1]),(global_waypoints_vis[s+1][0] - global_waypoints_vis[s][0]))
            quaternions = quaternion_from_euler(0, 0, theta)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.ARROW
            marker.scale.x = 0.5
            marker.scale.y = 0.05
            marker.scale.z = 0.15
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = global_waypoints_vis[s][0]
            marker.pose.position.y = global_waypoints_vis[s][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = quaternions[0]
            marker.pose.orientation.y = quaternions[1]
            marker.pose.orientation.z = quaternions[2]
            marker.pose.orientation.w = quaternions[3]
            marker.id = i
            sec_markers.markers.append(marker)

            marker_text = Marker()
            marker_text.header.frame_id = "map"
            marker_text.header.stamp = self.get_clock().now().to_msg()
            marker_text.type = marker_text.TEXT_VIEW_FACING
            _lane = self.sectors_params[f"Sector{i}"].get('lane', None)
            marker_text.text = (
                f"Start Sector {i}"
                + (" [STRAIGHT]" if self.sectors_params[f"Sector{i}"].get('is_straight', False) else "")
                + (f" [LANE: {_lane}]" if _lane not in LANE_NONE_VALUES else "")
            )
            marker_text.scale.z = 0.4
            marker_text.color.r = 0.2
            marker_text.color.g = 0.1
            marker_text.color.b = 0.1
            marker_text.color.a = 1.0
            marker_text.pose.position.x = global_waypoints_vis[s][0]
            marker_text.pose.position.y = global_waypoints_vis[s][1]
            marker_text.pose.position.z = 1.5
            marker_text.pose.orientation.x = 0.0
            marker_text.pose.orientation.y = 0.0
            marker_text.pose.orientation.z = 0.0436194
            marker_text.pose.orientation.w = 0.9990482
            marker_text.id = i + n_sectors
            sec_markers.markers.append(marker_text)
        self.sector_visualization_pub.publish(sec_markers)


def main():
    rclpy.init()
    node = SectorTuner()
    rclpy.spin(node)
    rclpy.shutdown()
