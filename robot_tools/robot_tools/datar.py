import numpy as np
from typing import Tuple


def remove_target(arr: np.ndarray, target) -> Tuple[np.ndarray, np.ndarray]:
    """删除一维数组中等于0的元素并返回原数组和被删除元素的索引"""
    return arr[arr != 0], np.where(arr == target)[0]


if __name__ == "__main__":
    test_array = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9])
    test_array, index = remove_target(test_array, 3)
    print(test_array), print(index)
