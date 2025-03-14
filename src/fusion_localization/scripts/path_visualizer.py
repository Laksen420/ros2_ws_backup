#!/usr/bin/env python3
import json
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse
from datetime import datetime
import glob

def load_path_data(filename):
    """Load path data from a JSON file."""
    with open(filename, 'r') as f:
        data = json.load(f)
    return data

def plot_path(data, output_file=None, show=True):
    """Plot the GPS paths with matplotlib."""
    # Create the figure
    plt.figure(figsize=(12, 10))
    
    # Plot raw path if available
    if data.get('raw_path'):
        x_raw = [p['x'] for p in data['raw_path']]
        y_raw = [p['y'] for p in data['raw_path']]
        plt.plot(x_raw, y_raw, 'r-', label='Raw GPS Path', alpha=0.7)
        plt.plot(x_raw[0], y_raw[0], 'ro', markersize=10, label='Start')
        
    # Plot filtered path if available
    if data.get('filtered_path'):
        x_filtered = [p['x'] for p in data['filtered_path']]
        y_filtered = [p['y'] for p in data['filtered_path']]
        plt.plot(x_filtered, y_filtered, 'b-', label='Filtered GPS Path', alpha=0.7)
        plt.plot(x_filtered[-1], y_filtered[-1], 'go', markersize=10, label='Current Position')
    
    # Add origin marker
    plt.plot(0, 0, 'ko', markersize=8, label='Origin')
    
    # Calculate the center of the path for info display
    if data.get('filtered_path'):
        center_x = np.mean(x_filtered)
        center_y = np.mean(y_filtered)
    else:
        center_x = np.mean(x_raw)
        center_y = np.mean(y_raw)
    
    # Add some info text
    plt.annotate(
        f"Origin: {data['origin']['latitude']:.6f}, {data['origin']['longitude']:.6f}\n"
        f"Timestamp: {data['timestamp']}",
        xy=(0.02, 0.02), xycoords='axes fraction',
        fontsize=10, bbox=dict(boxstyle="round,pad=0.5", fc="white", alpha=0.8)
    )
    
    # Add grid, labels, title, and legend
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    plt.title('GPS Path Visualization')
    plt.legend()
    
    # Make axes equal for proper scaling
    plt.axis('equal')
    
    # Add margin around the path
    plt.tight_layout()
    
    # Save if requested
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {output_file}")
    
    # Show if requested
    if show:
        plt.show()
    else:
        plt.close()

def analyze_path(data):
    """Perform basic analysis on the path data."""
    print("\n===== Path Analysis =====")
    
    # Prefer filtered path if available
    if data.get('filtered_path'):
        path = data['filtered_path']
        path_type = "Filtered"
    else:
        path = data['raw_path']
        path_type = "Raw"
    
    # Origin info
    print(f"Origin: {data['origin']['latitude']:.6f}°, {data['origin']['longitude']:.6f}°")
    print(f"Timestamp: {data['timestamp']}")
    
    # Basic stats
    print(f"\n{path_type} Path Statistics:")
    print(f"  Number of points: {len(path)}")
    
    # Calculate path length
    if len(path) >= 2:
        total_distance = 0
        for i in range(1, len(path)):
            dx = path[i]['x'] - path[i-1]['x']
            dy = path[i]['y'] - path[i-1]['y']
            segment_distance = np.sqrt(dx**2 + dy**2)
            total_distance += segment_distance
        
        print(f"  Total distance: {total_distance:.2f} meters")
        
        # Calculate average speed if timestamps are available
        if 'timestamp' in path[0] and 'timestamp' in path[-1]:
            start_time = path[0]['timestamp']
            end_time = path[-1]['timestamp']
            duration = end_time - start_time
            if duration > 0:
                avg_speed = total_distance / duration
                print(f"  Duration: {duration:.2f} seconds")
                print(f"  Average speed: {avg_speed:.2f} m/s ({avg_speed * 3.6:.2f} km/h)")
    
    # Calculate bounding box
    if path:
        min_x = min(p['x'] for p in path)
        max_x = max(p['x'] for p in path)
        min_y = min(p['y'] for p in path)
        max_y = max(p['y'] for p in path)
        
        print(f"\nBounding Box:")
        print(f"  X range: {min_x:.2f} to {max_x:.2f} meters (width: {max_x - min_x:.2f} meters)")
        print(f"  Y range: {min_y:.2f} to {max_y:.2f} meters (height: {max_y - min_y:.2f} meters)")
    
    print("\n=========================")

def list_path_files(directory="path_logs"):
    """List all path data files in the specified directory."""
    if not os.path.exists(directory):
        print(f"Directory {directory} not found.")
        return []
        
    # Find all JSON files
    json_files = glob.glob(os.path.join(directory, "*.json"))
    
    if not json_files:
        print(f"No path data files found in {directory}")
        return []
    
    # Sort by modification time (newest first)
    json_files.sort(key=os.path.getmtime, reverse=True)
    
    print(f"\nFound {len(json_files)} path data files:")
    for i, file in enumerate(json_files):
        timestamp = datetime.fromtimestamp(os.path.getmtime(file)).strftime("%Y-%m-%d %H:%M:%S")
        print(f"{i+1}. {os.path.basename(file)} - {timestamp}")
    
    return json_files

def main():
    parser = argparse.ArgumentParser(description="GPS Path Visualization and Analysis Tool")
    parser.add_argument("-f", "--file", help="Path to JSON file with path data")
    parser.add_argument("-l", "--list", action="store_true", help="List available path data files")
    parser.add_argument("-n", "--number", type=int, help="Select file number from list")
    parser.add_argument("-o", "--output", help="Output file for plot (e.g., path.png)")
    parser.add_argument("-d", "--directory", default="path_logs", help="Directory with path data files")
    parser.add_argument("--no-show", action="store_true", help="Don't display the plot window")
    
    args = parser.parse_args()
    
    # List files if requested
    if args.list or (not args.file and not args.number):
        files = list_path_files(args.directory)
        
        if not files:
            return
            
        # Automatically select the most recent file if no specific file is given
        if not args.file and not args.number:
            print("\nFor detailed options, use -h or --help")
            prompt = input("\nEnter file number to visualize (or press Enter for most recent): ")
            if prompt.strip():
                try:
                    file_idx = int(prompt) - 1
                    if 0 <= file_idx < len(files):
                        args.file = files[file_idx]
                    else:
                        print("Invalid file number.")
                        return
                except ValueError:
                    print("Invalid input.")
                    return
            else:
                args.file = files[0]  # Most recent file
    
    # Select file by number
    elif args.number:
        files = list_path_files(args.directory)
        if not files:
            return
            
        if 1 <= args.number <= len(files):
            args.file = files[args.number - 1]
        else:
            print(f"Invalid file number. Please select a number between 1 and {len(files)}.")
            return
    
    # Process the file
    if args.file:
        try:
            data = load_path_data(args.file)
            
            # Generate default output filename if needed
            if args.output is None and not args.no_show:
                base_name = os.path.splitext(os.path.basename(args.file))[0]
                args.output = f"{base_name}_plot.png"
            
            # Analyze path
            analyze_path(data)
            
            # Plot path
            plot_path(data, args.output, not args.no_show)
            
        except Exception as e:
            print(f"Error processing file: {e}")
    else:
        print("No file specified. Use -f/--file to specify a file or -l/--list to see available files.")

if __name__ == "__main__":
    main()
