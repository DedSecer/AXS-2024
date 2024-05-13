""" 目录处理相关函数 """


def remove_str_after_target(origin: str, target: str, include: bool = False):
    """从字符串中删除目标字符串之后的内容
    origin:原始字符串
    target:目标字符串
    include:为假则不删除target，否则删除，默认不删除
    """
    # 查找目标字符串在原始字符串中的位置
    position = origin.find(target)
    # 如果找到目标字符串
    if position != -1:
        # 删除要删除字符串之后的内容
        if include:
            modified = origin[:position]  # 删除target
        else:
            modified = origin[: position + len(target)]  # 不删除target
    else:
        # 如果未找到目标字符串，保持原始字符串不变
        modified = origin
        raise Exception("未找到指定字符串")
    return modified


def get_upper_path(path: str, n: int = 1) -> str:
    """获取向上n层的路径，默认n=1，即父目录
    path: the start path
    n: how many steps to go, set to 1 by default
    """
    try:
        for _ in range(n):
            path = path[: path.rfind("/")]
    except:
        raise Exception("错误：n > 目录最大级数")

    return path


def split_path(path: str, with_slash=False):
    paths = path.split("/")
    paths.pop(0)  # remove the first useless item
    if with_slash:
        for path in paths:
            path = "/".join(path)
    return paths


import os


def get_current_path() -> str:
    """获取当前文件所在的绝对路径"""
    root_path = os.path.dirname(os.path.abspath(__file__))
    return root_path


def create_dir(path: str):
    """如果目录不存在则创建目录"""
    if not os.path.exists(path):
        os.makedirs(path)


try:
    import rospkg
except:
    __ROS_PKG_NOT_FOUND__ = True
else:
    __ROS_PKG_NOT_FOUND__ = False

from typing import Union, Tuple


def get_ros_pkg_and_workspace_path(pkg_name, ws_name=None) -> Union[Tuple[str], str]:
    """获取指定ROS包和工作空间的绝对路径
    pkg_name: the name of the package
    ws_name: the name of the workspace, which is set to None by default
    and thus only return pkgname, ortherwise will return the tuple of
    (pkg_name, ws_name)
    """
    # 借助rospack工具和上述字符串处理函数找到包和工作空间的绝对路径
    rospack = rospkg.RosPack()
    PKG_DIRECTORY = rospack.get_path(pkg_name)  # 获取ros包路径（例如：***/graspnet_pkg）
    if ws_name is not None:
        WS_DIRECTORY = remove_str_after_target(
            PKG_DIRECTORY, ws_name
        )  # 通过包路径获取包所在工作空间路径（例如：***/graspnet_ws）
        return (PKG_DIRECTORY, WS_DIRECTORY)
    else:
        return PKG_DIRECTORY


if __name__ == "__main__":
    cp = get_current_path() + ":"
    up = get_upper_path(cp, 2) + ":"
    rp = get_ros_pkg_and_workspace_path("graspnet_pkg", "graspnet_ws")
    sp = split_path(cp)
    print(cp, up, rp, sp)
