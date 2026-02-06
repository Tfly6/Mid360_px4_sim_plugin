import sys
import numpy as np
import trimesh

def main():
    if len(sys.argv) < 3:
        print("Usage: python calc_inertia.py <path_to_dae> <mass>")
        sys.exit(1)

    mesh_path = sys.argv[1]
    mass = float(sys.argv[2])

    # 读取 mesh
    mesh = trimesh.load(mesh_path, force='mesh')
    if mesh.is_empty:
        raise ValueError("Mesh is empty or failed to load.")

    # 计算包围盒尺寸
    bounds = mesh.bounds  # [[minx,miny,minz],[maxx,maxy,maxz]]
    size = bounds[1] - bounds[0]
    Lx, Ly, Lz = size

    # 盒体惯量公式
    Ixx = (1/12) * mass * (Ly**2 + Lz**2)
    Iyy = (1/12) * mass * (Lx**2 + Lz**2)
    Izz = (1/12) * mass * (Lx**2 + Ly**2)

    print("Bounding box size (m):")
    print(f"  Lx={Lx:.6f}, Ly={Ly:.6f}, Lz={Lz:.6f}")
    print("Suggested inertia (kg*m^2):")
    print(f"  ixx={Ixx:.6e}")
    print(f"  iyy={Iyy:.6e}")
    print(f"  izz={Izz:.6e}")
    print("  ixy=ixz=iyz=0")

if __name__ == "__main__":
    main()