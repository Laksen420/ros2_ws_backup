#!/usr/bin/env python3
import os
import sys
import trimesh
import numpy as np
from ament_index_python.packages import get_package_share_directory

def find_center_of_model(mesh_file):
    try:
        mesh = trimesh.load(mesh_file)
        
        # Get bounding box
        bounds = mesh.bounds
        print("Model bounds:")
        print(f"X: {bounds[0][0]} to {bounds[1][0]}")
        print(f"Y: {bounds[0][1]} to {bounds[1][1]}")
        print(f"Z: {bounds[0][2]} to {bounds[1][2]}")
        
        # Calculate center
        center = mesh.centroid
        print(f"\nModel centroid (center point): {center}")
        
        # Calculate recommended offset 
        # (negative of center coordinates to move origin to center)
        print("\nRecommended URDF visual origin offset:")
        print(f"<origin xyz=\"{-center[0]:.4f} {-center[1]:.4f} {-center[2]:.4f}\" rpy=\"-1.57 0 0\"/>")
        
        print("\nRecommended joint origin offset (keep your Z value):")
        print(f"<origin xyz=\"{-center[0]:.4f} {-center[1]:.4f} -0.5\" rpy=\"0 0 0\"/>")
        
    except Exception as e:
        print(f"Error processing mesh: {e}")
        return None

if __name__ == "__main__":
    if len(sys.argv) > 1:
        mesh_file = sys.argv[1]
    else:
        pkg_dir = get_package_share_directory('forklift_description')
        mesh_file = os.path.join(pkg_dir, 'meshes', 'forklift.dae')
    
    find_center_of_model(mesh_file)
