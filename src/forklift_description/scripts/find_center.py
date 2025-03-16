#!/usr/bin/env python3

import os
import sys
import trimesh
import xml.etree.ElementTree as ET

def find_mesh_files_in_urdf(urdf_path):
    """Extract all mesh file paths from a URDF file."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # Find all mesh elements
    mesh_files = []
    for visual in root.findall(".//visual"):
        mesh_elem = visual.find(".//mesh")
        if mesh_elem is not None:
            filename = mesh_elem.get("filename")
            if filename:
                # Handle package:// URLs in URDF
                if filename.startswith("package://"):
                    parts = filename.split("/")
                    package_name = parts[2]
                    rel_path = "/".join(parts[3:])
                    
                    # Try to resolve the package path
                    base_path = os.path.expanduser("~/ros2_ws/src")
                    potential_path = os.path.join(base_path, package_name, rel_path)
                    
                    if os.path.exists(potential_path):
                        mesh_files.append((potential_path, visual))
                else:
                    # Direct path
                    if os.path.exists(filename):
                        mesh_files.append((filename, visual))
    
    return mesh_files

def analyze_mesh(mesh_path):
    """Calculate center of mass and bounding box for a mesh file."""
    try:
        print(f"Loading mesh: {mesh_path}")
        mesh = trimesh.load(mesh_path)
        
        # Get geometric center (centroid)
        centroid = mesh.centroid
        
        # Get bounding box
        bounds = mesh.bounds
        geometric_center = (bounds[0] + bounds[1]) / 2
        
        print(f"Mesh: {os.path.basename(mesh_path)}")
        print(f"  Centroid: {centroid}")
        print(f"  Bounding box min: {bounds[0]}")
        print(f"  Bounding box max: {bounds[1]}")
        print(f"  Geometric center: {geometric_center}")
        
        return centroid, bounds
    except Exception as e:
        print(f"Error analyzing mesh {mesh_path}: {e}")
        return None, None

def main():
    if len(sys.argv) < 2:
        print("Usage: python find_center_dae.py path/to/your.urdf")
        # If no arguments, try to find the common forklift file
        urdf_path = os.path.expanduser("~/ros2_ws/src/forklift_description/urdf/forklift_description.urdf")
        if not os.path.exists(urdf_path):
            print(f"Default file not found: {urdf_path}")
            return
        print(f"Using default URDF file: {urdf_path}")
    else:
        urdf_path = sys.argv[1]
    
    mesh_files = find_mesh_files_in_urdf(urdf_path)
    
    if not mesh_files:
        print("No mesh files found in the URDF!")
        # Try to directly analyze a known mesh file as fallback
        dae_path = os.path.expanduser("~/ros2_ws/src/forklift_description/meshes/forklift_opt4.dae")
        if os.path.exists(dae_path):
            print(f"Trying to directly analyze mesh: {dae_path}")
            centroid, bounds = analyze_mesh(dae_path)
            if centroid is not None:
                print("\nTo center this model, add to your URDF origin:")
                print(f"  <origin xyz=\"{-centroid[0]:.6f} {-centroid[1]:.6f} {-centroid[2]:.6f}\" rpy=\"0 0 0\"/>")
        return
    
    print(f"Found {len(mesh_files)} mesh references in the URDF.")
    
    for mesh_path, visual_elem in mesh_files:
        centroid, bounds = analyze_mesh(mesh_path)
        
        if centroid is not None:
            # Find parent link
            parent_link = None
            parent = visual_elem.getparent() if hasattr(visual_elem, 'getparent') else None
            while parent is not None:
                if parent.tag == 'link':
                    parent_link = parent.get('name')
                    break
                parent = parent.getparent() if hasattr(parent, 'getparent') else None
            
            link_name = "unknown link" if parent_link is None else f"link '{parent_link}'"
            print(f"\nFor {link_name}, add this origin to center the model:")
            print(f"  <origin xyz=\"{-centroid[0]:.6f} {-centroid[1]:.6f} {-centroid[2]:.6f}\" rpy=\"0 0 0\"/>")

    # Direct approach for known mesh if URDF parsing doesn't yield results
    if not mesh_files:
        dae_path = os.path.expanduser("~/ros2_ws/src/forklift_description/meshes/forklift_opt4.dae")
        if os.path.exists(dae_path):
            print(f"Trying direct approach with mesh: {dae_path}")
            analyze_mesh(dae_path)

if __name__ == "__main__":
    main()
