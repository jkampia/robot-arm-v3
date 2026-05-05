# Helper class for inverse and forward kinematics, motion planning

import base64
import math as m
import numpy as np
import json

from enum import Enum

from itertools import product

from colors import COLOR_RGB, pickRandomColor

np.set_printoptions(suppress=True, precision=2)


def _axisPlotSvg(points, axis_indices, title, x_label, y_label, x_limits=None, y_limits=None, width=360, height=360, preserveAspectRatio=False):
    marginLeft = 54
    marginRight = 34
    marginTop = 34
    marginBottom = 54

    xs = [point[axis_indices[0]] for point in points]
    ys = [point[axis_indices[1]] for point in points]

    if x_limits is None:
        minX, maxX = min(xs), max(xs)
        spanX = max(maxX - minX, 1.0)
        padX = spanX * 0.12
        minX -= padX
        maxX += padX
    else:
        minX, maxX = x_limits

    if y_limits is None:
        minY, maxY = min(ys), max(ys)
        spanY = max(maxY - minY, 1.0)
        padY = spanY * 0.12
        minY -= padY
        maxY += padY
    else:
        minY, maxY = y_limits

    availableWidth = width - marginLeft - marginRight
    availableHeight = height - marginTop - marginBottom

    if preserveAspectRatio:
        dataAspectRatio = (maxX - minX) / (maxY - minY)
        plotWidth = min(availableWidth, availableHeight * dataAspectRatio)
        plotHeight = plotWidth / dataAspectRatio
    else:
        plotSize = min(availableWidth, availableHeight)
        plotWidth = plotSize
        plotHeight = plotSize

    def project(x, y):
        px = marginLeft + ((x - minX) / (maxX - minX)) * plotWidth
        py = marginTop + plotHeight - ((y - minY) / (maxY - minY)) * plotHeight
        return px, py

    projectedPoints = [project(x, y) for x, y in zip(xs, ys)]
    polylinePoints = " ".join(f"{x:.1f},{y:.1f}" for x, y in projectedPoints)

    gridMarkup = []
    gridSpacing = (maxX - minX) / 4.0
    xGridCount = int(round((maxX - minX) / gridSpacing))
    yGridCount = int(round((maxY - minY) / gridSpacing))
    for step in range(xGridCount + 1):
        x = marginLeft + (step / xGridCount) * plotWidth
        gridMarkup.append(
            f'<line x1="{x:.1f}" y1="{marginTop}" x2="{x:.1f}" y2="{marginTop + plotHeight}" '
            f'stroke="#3d444d" stroke-width="1" />'
        )
    for step in range(yGridCount + 1):
        y = marginTop + (step / yGridCount) * plotHeight
        gridMarkup.append(
            f'<line x1="{marginLeft}" y1="{y:.1f}" x2="{marginLeft + plotWidth}" y2="{y:.1f}" '
            f'stroke="#3d444d" stroke-width="1" />'
        )

    jointColors = ["#f87171", "#fbbf24", "#34d399", "#60a5fa", "#a78bfa", "#f472b6"]
    jointMarkup = []
    for index, (x, y) in enumerate(projectedPoints):
        jointMarkup.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5.5" '
            f'fill="{jointColors[index % len(jointColors)]}" stroke="#111827" stroke-width="1.5" />'
        )

    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" rx="8" fill="#252a30" />
  <text x="{width / 2:.1f}" y="21" text-anchor="middle" fill="#f0f3f6" font-size="15" font-family="sans-serif">{title}</text>
  {"".join(gridMarkup)}
  <line x1="{marginLeft}" y1="{marginTop + plotHeight}" x2="{marginLeft + plotWidth}" y2="{marginTop + plotHeight}" stroke="#6b7280" />
  <line x1="{marginLeft}" y1="{marginTop}" x2="{marginLeft}" y2="{marginTop + plotHeight}" stroke="#6b7280" />
  <text x="{marginLeft + plotWidth / 2:.1f}" y="{height - 13}" text-anchor="middle" fill="#a9b1bb" font-size="12" font-family="sans-serif">{x_label}</text>
  <text x="16" y="{marginTop + plotHeight / 2:.1f}" text-anchor="middle" transform="rotate(-90 16 {marginTop + plotHeight / 2:.1f})" fill="#a9b1bb" font-size="12" font-family="sans-serif">{y_label}</text>
  <text x="{marginLeft}" y="{height - 29}" text-anchor="middle" fill="#77808b" font-size="10" font-family="sans-serif">{minX:.0f}</text>
  <text x="{marginLeft + plotWidth}" y="{height - 29}" text-anchor="middle" fill="#77808b" font-size="10" font-family="sans-serif">{maxX:.0f}</text>
  <text x="{marginLeft - 8}" y="{marginTop + plotHeight + 4}" text-anchor="end" fill="#77808b" font-size="10" font-family="sans-serif">{minY:.0f}</text>
  <text x="{marginLeft - 8}" y="{marginTop + 4}" text-anchor="end" fill="#77808b" font-size="10" font-family="sans-serif">{maxY:.0f}</text>
  <polyline points="{polylinePoints}" fill="none" stroke="#7dd3fc" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round" />
  {"".join(jointMarkup)}
</svg>'''


def _svgToBase64(svg):
    return base64.b64encode(svg.encode("utf-8")).decode("ascii")


def _withoutConsecutiveDuplicates(points, tolerance=1e-9):
    out = []
    for point in points:
        if not out:
            out.append(point)
            continue

        distance = np.linalg.norm(np.array(point) - np.array(out[-1]))
        if distance > tolerance:
            out.append(point)

    return out


def _toolDirectionFromJointAngles(jointAngles):
    theta1 = jointAngles[0]
    theta4World = jointAngles[1] + jointAngles[2] + jointAngles[3] - m.pi/2
    return np.array([
        m.cos(theta4World) * m.cos(theta1),
        m.cos(theta4World) * m.sin(theta1),
        m.sin(theta4World),
    ])


class ARM_5DOF:


    def __init__(self):
        
        linkLengths = [100, 100, 100, 100, 100]
        initialAngles = [0.0, m.pi/2, 0.0, 0.0, 0.0]

        self.linkLengths = linkLengths.copy()
        self.homeAngles = initialAngles.copy()

        self.jointAngles = initialAngles.copy()
        self.jointCoordinates = [[0.0, 0.0, 0.0], [], [], [], [], []] # default mm
        self.jointCoordinatesM = [[0.0, 0.0, 0.0], [], [], [], [], []] # non default m

        self.dhParams = {}
        self.dhParams["alpha"] = [m.pi/2, 0.0, 0.0, m.pi/2, 0.0]
        self.dhParams["d"] = [linkLengths[0], 0.0, 0.0, 0.0, linkLengths[3]]
        self.dhParams["a"] = [0.0, linkLengths[1], linkLengths[2], 0.0, 0.0]

        self.status = "ready"


    def setStatus(self, status):
        self.status = str(status)


    def setJointAnglesRad(self, jointAngles):
        if len(jointAngles) != 5:
            raise ValueError(f"Expected 5 joint angles, got {len(jointAngles)}")

        self.jointAngles = [float(angle) for angle in jointAngles]
        self.currentForwardKinematicPose()


    def setJointAnglesDeg(self, jointAnglesDeg):
        self.setJointAnglesRad([m.radians(float(angle)) for angle in jointAnglesDeg])


    def setPoseMmDeg(self, targetPose):
        if len(targetPose) != 5:
            raise ValueError(f"Expected 5 pose values, got {len(targetPose)}")

        self.setJointAnglesRad(self.solveIK([float(value) for value in targetPose]))


    def currentForwardKinematicPose(self):
        jointCoordinates = self.solveFK(self.jointAngles)
        self.jointCoordinates = [
            [float(value) for value in joint]
            for joint in jointCoordinates
        ]
        self.jointCoordinatesM = [
            [value / 1000.0 for value in joint]
            for joint in self.jointCoordinates
        ]

        endEffector = self.jointCoordinates[-1]
        theta4World = self.jointAngles[1] + self.jointAngles[2] + self.jointAngles[3] - m.pi/2
        return [
            endEffector[0],
            endEffector[1],
            endEffector[2],
            m.degrees(theta4World),
            m.degrees(self.jointAngles[4]),
        ]


    def stateToDict(self):
        currentPose = self.currentForwardKinematicPose()

        return {
            "type": "robot_state",
            
            "home_angles_rad": self.homeAngles,

            "joint_angles_rad": self.jointAngles,

            "joint_angles_deg": [
                m.degrees(angle)
                for angle in self.jointAngles
            ],

            "joint_coordinates_m": self.jointCoordinatesM,

            "fk_pose_mm_deg": currentPose,

            "link_plot_svgs": visualizeLinks(self)["plots"],

            "status": self.status,
        }
    

    def toJSON(self):
        return json.dumps(self.stateToDict(), indent=4)

    def printArrayPretty(self, array):
        print(' '.join(f"{val:6.2f}" for val in array))
        print("")
    
    
    def printMatrixPretty(self, matrix):
        for row in matrix:
            print(' '.join(f"{val:6.2f}" for val in row))
        print("")


    def extractCoordinates(self, tf_mat):
        # extract x, y, z position of a particular joint given its T0i tf matrix      
        return [tf_mat[0][3], tf_mat[1][3], tf_mat[2][3]]
    

    def tfMatrix(self, i):

        # extract variables from dict for verbosity
        theta = self.dhParams["theta"][i]
        alpha = self.dhParams["alpha"][i]
        a = self.dhParams["a"][i]
        d = self.dhParams["d"][i]

        # from standard layout of denavit-hartenburg matrix
        return np.array([[m.cos(theta), -m.sin(theta)*m.cos(alpha), m.sin(theta)*m.sin(alpha), a*m.cos(theta)], 
                         [m.sin(theta), m.cos(theta)*m.cos(alpha), -m.cos(theta)*m.sin(alpha), a*m.sin(theta)], 
                         [0.0, m.sin(alpha), m.cos(alpha), d], 
                         [0.0, 0.0, 0.0, 1.0]])


    def solveFK(self, jointAngles):

        # set 'theta' field of dh matrix
        self.dhParams["theta"] = jointAngles
        jointCoordinates = [[0.0, 0.0, 0.0], [], [], [], [], [], []] # default mm
        
        # generate tf matrices
        product = self.tfMatrix(0)
        jointCoordinates[1] = self.extractCoordinates(product)
        for i in range(1, 5):
            product = np.matmul(product, self.tfMatrix(i))
            jointCoordinates[i+1] = self.extractCoordinates(product) 

        toolDirection = _toolDirectionFromJointAngles(jointAngles)
        jointCoordinates[6] = (
            np.array(jointCoordinates[5], dtype=float)
            + toolDirection * float(self.linkLengths[4])
        ).tolist()
        
        #self.printMatrixPretty(self.jointCoordinatesM)
        return jointCoordinates
        
    
    # geometric 5DOF inverse kinematics solver
    # assumes that the target pose is in the form of [x, y, z, theta4, theta5]
    # where theta4 is given WRT world horizonal frame (xy plane)
    # example: wrist straight down would be theta4 = -90 degrees
    def solveIK(self, targetPose):
        # unpack inputs for clarity
        x, y, z, theta4, theta5 = targetPose
        l1, l2, l3, l4, l5 = self.linkLengths

        # convert to radians
        theta4 = m.radians(theta4)
        theta5 = m.radians(theta5)

        # base yaw
        theta1 = m.atan2(y, x)

        # effective wrist offset
        wrist_len = l4 + l5
        horz = wrist_len * m.cos(theta4)
        x4 = x - horz * m.cos(theta1)
        y4 = y - horz * m.sin(theta1)
        z4 = z - wrist_len * m.sin(theta4) - l1

        # distance from shoulder to wrist
        rsquared = x4**2 + y4**2 + z4**2

        # elbow
        cos_phi = (rsquared - l2**2 - l3**2) / (2.0 * l2 * l3)
        theta3 = m.acos(cos_phi)

        # shoulder
        alpha = m.asin(z4 / m.sqrt(rsquared))
        beta = m.atan(l3 * m.sin(theta3) / (l2 + l3 * m.cos(theta3)))
        theta2 = alpha + beta 

        # flip elbow angle to match arm orientation
        # AFTER it is used to calculate theta2
        theta3 *= -1 

        # apply offsets to theta4 as it is provided in world frame
        # NOT last link frame, and FK expects last link frame
        j4_offset = theta2 + theta3 
        theta4 -= j4_offset - m.pi/2

        return [theta1, theta2, theta3, theta4, theta5]



def visualizeLinks(robot):

    joint_coordinates = robot.solveFK(robot.jointAngles)
    plotPoints = _withoutConsecutiveDuplicates(joint_coordinates)
    totalLinkLength = sum(robot.linkLengths) + 50

    xyPlot = _axisPlotSvg(
        plotPoints,
        (0, 1),
        "Joint Links: X/Y",
        "x (mm)",
        "y (mm)",
        x_limits=(-totalLinkLength, totalLinkLength),
        y_limits=(-totalLinkLength, totalLinkLength),
    )
    xzPlot = _axisPlotSvg(
        plotPoints,
        (0, 2),
        "Joint Links: X/Z",
        "x (mm)",
        "z (mm)",
        x_limits=(-550, 550),
        y_limits=(0, totalLinkLength),
        width=640,
        height=360,
        preserveAspectRatio=True,
    )

    return {
        "plots": {
            "xy": _svgToBase64(xyPlot),
            "xz": _svgToBase64(xzPlot),
        },
        "joint_coordinates": joint_coordinates,
    }


def main():

    robot = ARM_5DOF()
    visualizeLinks(robot)

    


if __name__ == "__main__":
    main()
