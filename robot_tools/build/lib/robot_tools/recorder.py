""" 数据记录（加载）相关函数 """


import json


def json_process(file_path, write=None, log=False):
    """读取/写入json文件"""

    if write is not None:
        with open(file_path, "w") as f_obj:
            json.dump(write, f_obj)
        if log:
            print("写入数据为：", write)
    else:
        with open(file_path) as f_obj:
            write = json.load(f_obj)
        if log:
            print("加载数据为：", write)
    return write


def json_append(file_path, write, log=False):
    """向json文件中追加数据"""

    with open(file_path, "r+") as f_obj:
        data = json.load(f_obj)
        data.update(write)
        f_obj.seek(0)
        json.dump(data, f_obj)
    if log:
        print("追加数据为：", write)


def json_clear(file_path, log=False):
    """清空json文件"""

    with open(file_path, "w") as f_obj:
        json.dump({}, f_obj)
    if log:
        print("清空文件：", file_path)


def json_delete(file_path, key, log=False):
    """删除json文件中的某个键值对"""

    with open(file_path, "r+") as f_obj:
        data = json.load(f_obj)
        data.pop(key)
        f_obj.seek(0)
        json.dump(data, f_obj)
    if log:
        print("删除键值对：", key)


def json_update(file_path, key, value, log=False):
    """更新json文件中的某个键值对"""

    with open(file_path, "r+") as f_obj:
        data = json.load(f_obj)
        data[key] = value
        f_obj.seek(0)
        json.dump(data, f_obj)
    if log:
        print("更新键值对：", key, value)


if __name__ == "__main__":
    json_process("data.json", write={"a": 1, "b": 2}, log=True)
    json_process("data.json", log=True)
