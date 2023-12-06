import json
from copy import deepcopy


class OdPair(object):
    """
    class for O-D pairs
    """

    def __init__(self):
        # origin and destination data
        self.OD_id = None
        self.origin = None
        self.destination = None

        # path data
        self.path_dict = {}

        # flow data
        self.flow_rate = 0

    def build_od_pair(self, origin, destination, flow_rate):
        self.origin, self.destination = origin, destination
        self.OD_id = self.origin + '->' + self.destination
        self.flow_rate = flow_rate

    def add_path(self, path):
        """
        add a path to the OD pair, skip if already exists

        :param path:
        :return:
        """
        # skip this action if the path already exists
        exist_path = self._check_existing_path(path.edge_list)
        if exist_path is None:
            self.path_dict[path.path_id] = path
            return path.path_id
        else:
            return exist_path

    def _check_existing_path(self, edges):
        exist_flag = None
        for path_id, path in self.path_dict.items():
            path_edges = path.edge_list
            if edges == path_edges:
                return path_id
        return exist_flag

    def __str__(self):
        output_dict = deepcopy(self.__dict__)
        del output_dict["path_dict"]
        output_dict["path_dict(k)"] = list(self.path_dict.keys())
        # print(output_dict)
        return json.dumps(output_dict, indent=2)

    def __len__(self):
        """
        return the number of paths

        :return:
        """
        return len(self.path_dict)


class Path(object):
    """
    class for paths
    """

    def __init__(self):
        self.path_id = None
        # laneset information
        self.edge_list = []
        # belonged OD pair
        self.belonged_OD = None
        # path flow
        self.flow = 0
        # path travel time
        self.travel_time = 0

    def build_path(self, odpair, od_path_index, edge_list):
        self.edge_list = edge_list
        self.belonged_OD = odpair
        self.path_id = odpair + '@' + str(od_path_index)

    def update_path_travel_time(self, travel_time):
        self.travel_time = travel_time

    def __str__(self):
        return json.dumps(self.__dict__, indent=2)


