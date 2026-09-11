"""Build one explicit selection transaction using standard ROS interfaces."""
import math


def make_selection_request(x, y, frame, stamp, fault_epoch):
    """Use the server's observed epoch, never guess or advance it locally."""
    if (not all(type(v) in (int, float) and math.isfinite(v) for v in (x, y))
            or not isinstance(frame, str) or not frame
            or type(fault_epoch) is not int or not 0 < fault_epoch < 2**63
            or not 0 <= stamp.sec < 2**31 or not 0 <= stamp.nanosec < 10**9):
        raise ValueError('Invalid selection coordinates, frame, stamp or server epoch')
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParametersAtomically
    values = dict(target_x=float(x), target_y=float(y), frame_id=frame,
                  stamp_sec=int(stamp.sec), stamp_nanosec=int(stamp.nanosec),
                  fault_epoch=fault_epoch)
    request = SetParametersAtomically.Request()
    request.parameters = [Parameter(name, value=value).to_parameter_msg()
                          for name, value in values.items()]
    return request


def make_enable_request(stamp, fault_epoch, selection_id):
    """Enable only the explicitly observed selection in its current fault epoch."""
    if (type(fault_epoch) is not int or not 0 < fault_epoch < 2**63
            or type(selection_id) is not int or not 0 < selection_id < 2**63
            or not 0 <= stamp.sec < 2**31 or not 0 <= stamp.nanosec < 10**9):
        raise ValueError('Invalid enable timestamp or generation')
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParametersAtomically
    values = dict(stamp_sec=int(stamp.sec), stamp_nanosec=int(stamp.nanosec),
                  fault_epoch=fault_epoch, selection_id=selection_id)
    request = SetParametersAtomically.Request()
    request.parameters = [Parameter(name, value=value).to_parameter_msg()
                          for name, value in values.items()]
    return request
