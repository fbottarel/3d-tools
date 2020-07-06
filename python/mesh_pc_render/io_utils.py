import numpy as np
import os


def savePCNPtoPCD(points, filename):
    """
    Save a numpy point vector to PCD format.

    Parameters
    ----------
    points: (n, 3) float
        Points to save

    filename: str
        Name of the output file (with or without extension) as full path

    Returns
    ----------
    success: bool
        True if the operation was successful

    """

    if os.path.splitext(filename)[1] != ".pcd":
        filename += ".pcd"

    success = False

    pcd_file_header = (
    "# .PCD v.7 - Point Cloud Data file format\n"
    "VERSION .7\n"
    "FIELDS x y z\n"
    "SIZE 4 4 4\n"
    "TYPE F F F\n"
    "COUNT 1 1 1\n"
    "WIDTH " + str(points.shape[0])+"\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS " + str(points.shape[0])+"\n"
    "DATA ascii")

    try:
        with open(filename, 'w') as handle:
            np.savetxt(handle, points, header=pcd_file_header, comments='')
            success = True
    except:
        print("Could not open file " + filename)

    return success


def savePCNPtoXYZ(points, filename):
    """
    Save a numpy point vector to raw text XYZ format.

    Parameters
    ----------
    points: (n, 3) float
        Points to save

    filename: str
        Name of the output file (with or without extension) as full path

    Returns
    ----------
    success: bool
        True if the operation was successful

    """

    if os.path.splitext(filename)[1] != ".xyz":
        filename += ".xyz"

    success = False

    try:
        with open(filename, 'w') as handle:
            np.savetxt(handle, points)
            success = True
    except:
        print("Could not open file " + filename)

    return success

def savePCNPtoOFF(points, filename):
    """
    Save a numpy point vector to text OFF file.

    Parameters
    ----------
    points: (n, 3) float
        Points to save

    filename: str
        Name of the output file (with or without extension) as full path

    Returns
    ----------
    success: bool
        True if the operation was successful

    """

    if os.path.splitext(filename)[1] != ".off":
        filename += ".off"

    success = False

    off_file_header = (
    "OFF\n"
    + str(points.shape[0]) + " 0 0")

    try:
        with open(filename, 'w') as handle:
            np.savetxt(handle, points, header=off_file_header, comments='')
            success = True
    except:
        print("Could not open file " + filename)

    return success