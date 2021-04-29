#!/usr/bin/env python
"""
Module for parsing String message from vision_pipeline/data topic.
"""

def cleanString(originalString, charsToReplace):
    """Remove all unnecessary characters in received string."""

    for char in charsToReplace:
        cleanedString = originalString.replace(char, '')

    return cleanedString

def findIndexes(string, keywords):
    """Find starting indexes of all keywords."""

    # Find indexes of 'class_name' substrings:
    idxClassName = [i for i in range(len(string)) if string.startswith(keywords[0], i)]
    # Find indexes of 'score' substrings (after each class_name comes score):
    idxScore = [i for i in range(len(string)) if string.startswith(keywords[1], i)]
    # Find indexes of 'obb_corners' substring:
    idxCorners = [i for i in range(len(string)) if string.startswith(keywords[2], i)]
    # Find indexes of 'obb_center' substrings:
    idxCenter = [i for i in range(len(string)) if string.startswith(keywords[3], i)]
    # Find indexes of 'obb_rot_quat' substrings:
    idxQuat = [i for i in range(len(string)) if string.startswith(keywords[4], i)]

    return idxClassName, idxScore, idxCorners, idxCenter, idxQuat

def parseRawCoordinates(string, startIndex, endIndex, keyword):
    """Parse coordinate values for centers and corners."""

    rawValues = []
    for idx in range(len(startIndex)):
        points = string[startIndex[idx] + len(keyword) + 3 : endIndex[idx] - 3]
        # Split by comma and get rid of all [ and ]:
        points = points.replace('[', '')
        points = points.replace(']', '')
        points = points.split(',')
        # Convert to float:
        for point in points:
            point = float(point)
            rawValues.append(point)

    return rawValues

def parseClassNames(string, idxClassName, idxScore, keyword):
    """Parse all class names into a classNames list."""

    classNames = []
    for idx in range(len(idxClassName)):
        className = string[idxClassName[idx] + len(keyword) + 4 : idxScore[idx] - 4]
        classNames.append(className)

    return classNames

def parseCorners(string, idxCorners, idxCenter, keyword, coordOffset):
    """Parse and combine raw corner values into list of tuples."""

    rawCorners = parseRawCoordinates(string, idxCorners, idxCenter, keyword)
    cornerCoords = []
    for i in range(len(rawCorners)/10):
        cornerCoords.append(((rawCorners[10*i], rawCorners[10*i+1]), 
                            (rawCorners[10*i+2], rawCorners[10*i+3]),
                            (rawCorners[10*i+4], rawCorners[10*i+5]),
                            (rawCorners[10*i+6], rawCorners[10*i+7]),
                            (rawCorners[10*i+8], rawCorners[10*i+9])))

    return cornerCoords

def parseCenters(string, idxCenter, idxQuat, keyword, coordOffset):
    """Parse and combine raw center values into list of tuples."""

    rawCenters = parseRawCoordinates(string, idxCenter, idxQuat, keyword)
    centerCoords = []
    for i in range(len(rawCenters) / 2):
        centerCoords.append((rawCenters[2*i], rawCenters[2*i+1]))

    return centerCoords

def parseQuaternions(string, idxQuat, idxClassName, keyword):
    """Parse and combine quaternion values into list of tuples."""

    quatsRaw = []
    quatsVals = []
    for idx in range(len(idxQuat)):
        if (idx < len(idxQuat) - 1):
            quats = string[idxQuat[idx] + len(keyword) + 3 : idxClassName[idx + 1] - 5]
        else:
            quats = string[idxQuat[idx] + len(keyword) + 3 : -4]
        quats = quats.replace('[', '')
        quats = quats.replace(']', '')
        quats = quats.split(',')
        for quat in quats:
            quat = float(quat)
            quatsRaw.append(quat)
    for i in range(len(quatsRaw) / 4):
        quatsVals.append((quatsRaw[4*i], quatsRaw[4*i+1], quatsRaw[4*i+2], quatsRaw[4*i+3]))
        
    return quatsVals