import pickle
import geopy.distance

def load_gps_path(path_file='gps_path.pkl'):
    with open(path_file, 'rb') as f:
        return pickle.load(f)

def find_next_gps_point(current_lat, current_lon, gps_path):
    if not gps_path:
        return None
    distances = [geopy.distance.distance((current_lat, current_lon), p).meters for p in gps_path]
    min_idx = distances.index(min(distances))
    return gps_path[min(min_idx+1, len(gps_path)-1)]
