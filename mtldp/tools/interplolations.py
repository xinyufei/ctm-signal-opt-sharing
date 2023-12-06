def get_traverse_x(x_list, y_list, y_val, slope=None):
    """
    get the arrival distance according to certain distance value
    :param x_list:
    :param y_list:
    :param y_val:
    :param slope:
    :return:
    """
    traverse_x = None
    if (y_list[0] - y_val) * (y_list[-1] - y_val) > 0:
        if y_list[0] - y_val > 0:
            if slope is None:
                return traverse_x
            else:
                traverse_x = x_list[0] - (y_list[0] - y_val) / slope
                return traverse_x
        else:
            if slope is None:
                return traverse_x
            else:
                traverse_x = x_list[-1] + (y_val - y_list[-1]) / slope
                return traverse_x
    else:
        for idx in range(len(y_list) - 1):
            local_y = y_list[idx]
            local_x = x_list[idx]

            next_y = y_list[idx + 1]
            next_x = x_list[idx + 1]

            current_diff = local_y - y_val
            next_diff = next_y - y_val

            if abs(next_diff) < 0.05:
                return [next_x, True]

            if (next_diff > 0) and (current_diff < 0):
                traverse_x = local_x + (y_val - local_y) / \
                             (next_y - local_y) * (next_x - local_x)
                return traverse_x
    return traverse_x
