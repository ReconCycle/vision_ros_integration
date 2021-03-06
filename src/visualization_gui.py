#! /usr/bin/env python

from kivy.app import App
from kivy.lang import Builder
from kivy.properties import StringProperty
from kivy.properties import NumericProperty
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.popup import Popup
from kivy.uix.gridlayout import GridLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
from kivy.uix.scrollview import ScrollView
from kivy.core.window import Window
from kivy.uix.checkbox import CheckBox

import os
import os.path
import socket
import yaml
import math
import requests

import rospy
from rospy import Time
from visualization_msgs.msg import Marker
from robot_module_msgs.srv import StlFileManager
import tf


# Global variables
global tablesDict
global tablesNames
tablesDict = {}
tablesNames = []

# Class for scanning ips and hostnames.
class NetworkScanner():
    # ADD GLASS API CALL HERE
    def __init__(self):
        self.apiUrl = 'http://reconcycle-dev1.ijs.si:3000/api/get_active_leases'
    
    # Get Ip addresses in defined range.
    def getHostnames(self):
        availableHostnames = []
        apiResponse = requests.get(self.apiUrl)
        ipsDict = apiResponse.json()
        for ip in ipsDict:
            availableHostnames.append(ipsDict[ip]['host'])

        return availableHostnames
        

# Widget classes.
class ScrollBar(ScrollView):
    def __init__(self):
        super(ScrollBar, self).__init__()
        self.size_hint = (1, 0.7)
        self.pos_hint = {'center_x': 0.5, 'center_y': 0.5}

class AddButton(Button):
    def __init__(self, buttonId, buttonText, buttonPosHint, buttonSizeHint, buttonSize):
        super(AddButton, self).__init__()
        self.id = buttonId
        self.text = buttonText
        self.size_hint = (buttonSizeHint[0], buttonSizeHint[1])
        self.size = (buttonSize[0], buttonSize[1])
        self.pos_hint = {'center_x': buttonPosHint[0], 'center_y': buttonPosHint[1]}

class PopupWindow(Popup):
    def __init__(self, popupTitle, popupContent, popupSize = (200, 200)):
        super(PopupWindow, self).__init__()
        self.title = popupTitle
        self.content = popupContent
        self.size_hint = (None, None)
        self.size = popupSize

class TextInput(TextInput):
    def __init__(self, inputId, inputSizeHint, inputSize, inputHintText, inputPosHint, inputFontSize):
        super(TextInput, self).__init__()
        self.id = inputId
        self.size_hint = (inputSizeHint[0], inputSizeHint[1])
        self.size = (inputSize[0], inputSize[1])
        self.hint_text = inputHintText
        self.pos_hint = {'center_x': inputPosHint[0], 'center_y': inputPosHint[1]}
        self.font_size = inputFontSize
        self.multiline = False

class Label(Label):
    def __init__(self, labelId, labelText, labelTextSize, labelSizeHint, labelWidth):
        super(Label, self).__init__()
        self.id = labelId
        self.text = labelText
        self.text_size = labelTextSize
        self.size_hint = labelSizeHint
        self.width = labelWidth

class Checkbox(CheckBox):
    def __init__(self, isActive):
        super(Checkbox, self).__init__()
        self.active = isActive

# Class for text input validation and writing into dict.
class InputValidator():

    # Write into dict if text is valid or show popup if it's not.
    def onXyrTextValidation(self, textInput):
        global tablesNames
        global tablesDict

        for tableName in tablesNames:
            if tableName in textInput.id and len(tableName) == len(textInput.id) - 2:
                if self.validateInput(textInput.text):

                    if '_x' in textInput.id:
                        tablesDict[tableName][0] = {'x': int(textInput.text)}
                    if '_y' in textInput.id:
                        tablesDict[tableName][1] = {'y': int(textInput.text)}
                    if '_r' in textInput.id:
                        tablesDict[tableName][2] = {'r': int(textInput.text)}                

                else:
                    popupContent = Label('popupLabel', 'Invalid input!', (100, 20), (None, 1), 200)
                    popup = PopupWindow('Alert Window', popupContent, (200, 125))
                    popup.open()

    def onStlTextValidation(self, textInput):
        global tablesNames
        global tablesDict

        stlPath = '/ros_ws/src/reconcycle_visualization/reconcycle_description/meshes/'

        for tableName in tablesNames:
            if tableName in textInput.id:
                if os.path.isfile(stlPath + str(textInput.text) + '.stl'):
                    tablesDict[tableName][3] = {'stl' : str(textInput.text)}
                else:
                    popupContent = Label('popupLabel', 'Wrong stl file name!', (150, 40), (None, 1), 200)
                    popup = PopupWindow('Alert Window', popupContent, (200, 125))
                    popup.open()

    # Validate text input - it must be integer.
    def validateInput(self, input):
        try:
            return int(input), True
        except:
            return False

# Inner layout contains labels with hostnames and text inputs for positions.
class InnerLayout(GridLayout):

    def __init__(self):
        super(InnerLayout, self).__init__()
        self.cols = 6
        self.row_force_default = True
        self.row_default_height = 30
        self.spacing = 5
        self.size_hint_y = None
        self.inputValidator = InputValidator()
        self.innerWidgets = []
        self.addTableCounter = 0
        self.stlFileReader = StlFileReader()

    # Method for creating inner layout from scanned hostnames.
    def createFromScan(self):
        global tablesDict
        global tablesNames

        if len(tablesNames) > 0:
            for tableName in tablesNames:
                tableLabel = Label(tableName + '_label', tableName, (200, 20), (None, 1), 250)
                xPosInput = TextInput(tableName + '_x', (None, 1), (100, 0), 'Enter x', (None, None), 10)
                yPosInput = TextInput(tableName + '_y', (None, 1), (100, 0), 'Enter y', (None, None), 10)
                rotInput = TextInput(tableName + '_r', (None, 1), (100, 0), 'Enter rotation', (None, None), 10)
                stlFilenameInput = TextInput(tableName + '_stl', (None, 1), (100, 0), 'Enter stl file name', (None, None), 10)
                stlFilenameInput.text = self.stlFileReader.getStlFilename(tableName)
                removeTableCheckbox = Checkbox(False)

                xPosInput.bind(on_text_validate = self.inputValidator.onXyrTextValidation)
                yPosInput.bind(on_text_validate = self.inputValidator.onXyrTextValidation)
                rotInput.bind(on_text_validate = self.inputValidator.onXyrTextValidation)
                stlFilenameInput.bind(on_text_validate = self.inputValidator.onStlTextValidation)

                self.add_widget(tableLabel)
                self.add_widget(xPosInput)
                self.add_widget(yPosInput)
                self.add_widget(rotInput)
                self.add_widget(stlFilenameInput)
                self.add_widget(removeTableCheckbox)
                self.innerWidgets.append((tableLabel, xPosInput, yPosInput, rotInput, stlFilenameInput, removeTableCheckbox))

                tablesDict[tableName] = [{}, {}, {}, {}]
                if (stlFilenameInput.text):
                    tablesDict[tableName] = [{}, {}, {}, {'stl' : str(stlFilenameInput.text)}]
        else:
            popupContent = Label('popupLabel', 'Cannot find connected tables!', (250, 40), (None, 1), 300)
            popup = PopupWindow('Alert Window', popupContent, (300, 125))
            popup.open()

    # Method for creating inner layout from loaded hostnames (and values).
    def createFromLoad(self):
        global tablesDict
        global tablesNames

        for tableName in tablesNames:
            tableLabelId = tableName + '_label'
            tableLabel = Label(tableLabelId, tableName, (200, 20), (None, 1), 250)
            xPosInput = TextInput(tableName + '_x', (None, 1), (100, 0), '', (None, None), 10)
            try:
                xPosInput.text = str(tablesDict[tableName][0]['x'])
            except:
                xPosInput.hint_text = 'Enter x'
            yPosInput = TextInput(tableName + '_y', (None, 1), (100, 0), '', (None, None), 10)
            try:
                yPosInput.text = str(tablesDict[tableName][1]['y'])
            except:
                yPosInput.hint_text = 'Enter y'
            rotInput = TextInput(tableName + '_r', (None, 1), (100, 0), '', (None, None), 10)
            try:
                rotInput.text = str(tablesDict[tableName][2]['r'])
            except:
                rotInput.hint_text = 'Enter rotation'
            stlFilenameInput = TextInput(tableName + '_stl', (None, 1), (100, 0), '', (None, None), 10)
            try:
                stlFilenameInput.text = tablesDict[tableName][3]['stl']
            except:
                stlFilenameInput.hint_text = 'Enter stl file name'
            removeTableCheckbox = Checkbox(False)

            xPosInput.bind(on_text_validate = self.inputValidator.onXyrTextValidation)
            yPosInput.bind(on_text_validate = self.inputValidator.onXyrTextValidation)
            rotInput.bind(on_text_validate = self.inputValidator.onXyrTextValidation)
            stlFilenameInput.bind(on_text_validate = self.inputValidator.onStlTextValidation)

            self.add_widget(tableLabel)
            self.add_widget(xPosInput)
            self.add_widget(yPosInput)
            self.add_widget(rotInput)
            self.add_widget(stlFilenameInput)
            self.add_widget(removeTableCheckbox)
            self.innerWidgets.append((tableLabel, xPosInput, yPosInput, rotInput, stlFilenameInput, removeTableCheckbox))

        return self
    
    # Add additional non-connected table.
    def addTable(self, tableName):

        tableLabel = Label(tableName + '_label', tableName, (200, 20), (None, 1), 250)
        xPosInput = TextInput(tableName + '_x', (None, 1), (100, 0), 'Enter x', (None, None), 10)
        yPosInput = TextInput(tableName + '_y', (None, 1), (100, 0), 'Enter y', (None, None), 10)
        rotInput = TextInput(tableName + '_r', (None, 1), (100, 0), 'Enter rotation', (None, None), 10)
        stlFilenameInput = TextInput(tableName + '_stl', (None, 1), (100, 0), 'Enter stl file name', (None, None), 10)
        removeCheckbox = Checkbox(False)

        xPosInput.bind(on_text_validate = self.inputValidator.onXyrTextValidation)
        yPosInput.bind(on_text_validate = self.inputValidator.onXyrTextValidation)
        rotInput.bind(on_text_validate = self.inputValidator.onXyrTextValidation)
        stlFilenameInput.bind(on_text_validate = self.inputValidator.onStlTextValidation)

        self.add_widget(tableLabel)
        self.add_widget(xPosInput)
        self.add_widget(yPosInput)
        self.add_widget(rotInput)
        self.add_widget(stlFilenameInput)
        self.add_widget(removeCheckbox)
        self.innerWidgets.append((tableLabel, xPosInput, yPosInput, rotInput, stlFilenameInput, removeCheckbox))
        self.height += 50

    def openTableNamePopup(self, _):
        popupContent = FloatLayout()
        self.popupTableNameTextInput = TextInput('table_name_input', (1, None), (0, 30), '', (0.5, 0.85), 10)
        self.popupTableNameTextInput.hint_text = 'Enter table name'
        self.popupTableNameTextInput.bind(on_text_validate = self.onTableNameValidation)
        self.popupTableNameTextInput.bind(focus = self.onTableNameInputFocus)
        self.popupTableNameLabel = Label('text_name_popup_label', '', (150, 20), (None, 1), 250)
        self.popupTableNameLabel.pos_hint = {'center_x': 0.5, 'center_y': 0.5}
        self.popupTableNameButton = AddButton('tableNamePopupButton', 'Confirm', (0.5, 0.2), (None, None), (100, 40))
        self.popupTableNameButton.bind(on_press = self.onTableNameValidation)
        popupContent.add_widget(self.popupTableNameTextInput)
        popupContent.add_widget(self.popupTableNameButton)
        popupContent.add_widget(self.popupTableNameLabel)
        self.popupTableName = PopupWindow('Table Name Window', popupContent)
        self.popupTableName.open()
    
    def onTableNameValidation(self, _):
        global tablesDict
        global tablesNames
        innerLayout = InnerLayout()
        if self.popupTableNameTextInput.text in tablesDict:
            self.popupTableNameLabel.text = 'Table already exists!'
        else:
            tablesDict[str(self.popupTableNameTextInput.text)] = [{}, {}, {}, {}]
            tablesNames.append(str(self.popupTableNameTextInput.text))
            self.addTable(self.popupTableNameTextInput.text)
            self.popupTableName.dismiss()

    def onTableNameInputFocus(self, _, value):
        if value:
            self.popupTableNameLabel.text = ''
            self.popupTableNameTextInput.text = ''
    
    # Remove selected tables.
    def removeTable(self, _):
        global tablesNames
        global tablesDict

        widgetsToRemove = []
        # Remove widgets from screen and from dict
        for widgetTuple in self.innerWidgets:
            if widgetTuple[-1].active:
                widgetsToRemove.append(widgetTuple)
                self.height -= 50
                tablesDict.pop(widgetTuple[0].text)
                tablesNames.remove(widgetTuple[0].text)
                for widget in widgetTuple:
                    self.remove_widget(widget)
        
        # Remove widgets from innerWidgets array
        for widgetTuple in widgetsToRemove:
            self.innerWidgets.remove(widgetTuple)

    # Clear all widgets in inner layout.
    def removeInnerWidgets(self):
        self.addTableCounter = 0
        for widgetTuple in self.innerWidgets:
            for widget in widgetTuple:
                self.remove_widget(widget)

class YamlParser():

    def __init__(self):
        self.innerLayout = InnerLayout()
        self.popupTextInput = TextInput('yamlPopupInput', (1, None), (0, 30), '', (0.5, 0.85), 10)
        self.popupButton = AddButton('yamlPopupButton', '', (0.5, 0.2), (None, None), (100, 40))
        self.popupContent = FloatLayout()
        self.popupContent.add_widget(self.popupTextInput)
        self.popupContent.add_widget(self.popupButton)
        self.popup = PopupWindow('', self.popupContent)
        self.yamlFilePath = '/ros_ws/src/vision_ros_integration/config/cell_config/'

    # Open popup for saving yaml file.
    def openSaveYamlPopup(self, _):
        self.popupTextInput.unbind(on_text_validate = self.loadYamlFile)
        self.popupButton.unbind(on_press = self.loadYamlFile)
        self.popupTextInput.hint_text = 'Enter file name'
        self.popupButton.text = 'Save Yaml'
        self.popupTextInput.bind(on_text_validate = self.saveYamlFile)
        self.popupButton.bind(on_press = self.saveYamlFile)
        self.popup.title = 'Save Yaml File'
        self.popup.open()

    # Save yaml file.
    def saveYamlFile(self, _):
        self.popup.dismiss()
        yamlFileName =  self.yamlFilePath + self.popupTextInput.text
        self.popupTextInput.text = ''
        with open(str(yamlFileName) + '.yaml', 'w') as outputFile:
            yaml.dump(tablesDict, outputFile, default_flow_style = False)

    # Open popup for loading yaml file.
    def openLoadYamlPopup(self):
        self.popupTextInput.unbind(on_text_validate = self.saveYamlFile)
        self.popupButton.unbind(on_press = self.saveYamlFile)
        self.popupTextInput.hint_text = 'Enter file path'
        self.popupButton.text = 'Load Yaml'
        self.popupTextInput.bind(on_text_validate = self.loadYamlFile)
        self.popupButton.bind(on_press = self.loadYamlFile)
        self.popup.title = 'Load Yaml File'
        self.popup.open()

        return self.innerLayout

    # Load yaml file.
    def loadYamlFile(self, _):
        global tablesNames
        global tablesDict

        self.popup.dismiss()
        yamlFileName = self.yamlFilePath + self.popupTextInput.text
        self.popupTextInput.text = ''
        try:
            tablesDict = yaml.load(file(yamlFileName + '.yaml', 'r'))
            tablesNames = tablesDict.keys()
            self.innerLayout.createFromLoad()
        except:
            alertContent = Label('alertLabel', 'No such yaml file!', (180, 20), (None, 1), 200)
            fileAlertPopup = PopupWindow('Alert Window', alertContent, (200, 125))
            fileAlertPopup.open()

class RvizVisualization():

    def __init__(self):
        self.transformBroadcaster = tf.TransformBroadcaster()
        self.markerPublisher = rospy.Publisher('/visualization_marker', Marker, queue_size = 10)
        self.tableWidth = 0.6
    
    def visualizeTables(self, _):
        global tablesDict
        global tablesNames

        self.refreshMarkers()
        for tableName in tablesNames:
            self.sendTransformation(tableName)
            self.drawTableMarker(tableName)

    def sendTransformation(self, tableName):
        try:
            x = tablesDict[tableName][0]['x']
        except:
            x = 0
        try:
            y = tablesDict[tableName][1]['y']
        except:
            y = 0
        try:
            r = tablesDict[tableName][2]['r']
        except:
            r = 0
        translation = (x * self.tableWidth, y * self.tableWidth, 0.0)
        eulers = (0.0, 0.0, r * math.pi/180)
        quaternions = tf.transformations.quaternion_from_euler(eulers[0], eulers[1], eulers[2])
        self.transformBroadcaster.sendTransform(translation, quaternions, Time.now(), tableName, 'world')

    def drawTableMarker(self, tableName):
        global tablesDict

        meshResourcePath = 'package://reconcycle_description/meshes/'

        marker = Marker()
        marker.header.frame_id = tableName
        marker.header.stamp = Time.now()
        marker.ns = tableName
        marker.id = 1
        marker.type = marker.MESH_RESOURCE

        marker.mesh_use_embedded_materials = True
        marker.action = marker.ADD
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        marker.pose.orientation.x = 1.0
        marker.pose.orientation.w = 1.0

        try:
            marker.mesh_resource = meshResourcePath + tablesDict[tableName][3]['stl'] + '.stl' 
        except:
            marker.mesh_resource = meshResourcePath + 'table.stl'

        self.markerPublisher.publish(marker)       

        
    
    def refreshMarkers(self):
        marker = Marker()
        marker.action = marker.DELETEALL
        self.markerPublisher.publish(marker)

class StlFileReader():
    def __init__(self):
        self.serviceName = 'stl_file_service_'
    
    def getStlFilename(self, tableName):
        fullServiceName = self.serviceName + tableName
        # rospy.wait_for_service(fullServiceName)
        stlFilename = ''
        try:
            stlService = rospy.ServiceProxy(fullServiceName, StlFileManager)
            serviceResponse = stlService(tableName)
            stlFilename = serviceResponse.stlFilename
        except:
            pass
        
        return stlFilename

class Controller(FloatLayout):

    actionLabelText = StringProperty(defaultvalue = '')
    actionLabelPosX = NumericProperty(defaultvalue = 0.5)
    actionLabelPosY = NumericProperty(defaultvalue = 0.5)
    scanButtonPosX = NumericProperty(defaultvalue = 0.3)
    scanButtonPosY = NumericProperty(defaultvalue = 0.5)
    loadButtonPosX = NumericProperty(defaultvalue = 0.7)
    loadButtonPosY = NumericProperty(defaultvalue = 0.5)

    def __init__(self):
        super(Controller, self).__init__()
        self.networkScanner = NetworkScanner()
        self.scrollView = ScrollBar()
        self.innerLayout = InnerLayout()
        self.yamlParser = YamlParser()
        self.inputValidator = InputValidator()
        self.rvizVizualization = RvizVisualization()

    # Method called when button 'Scan' is pressed.
    def scanStart(self):
        self.actionLabelText = 'Scanning...'
    

    # Method called when button 'Scan' is released.
    def scanCompleted(self):
        global tablesNames

        self.removeWidgets()
        tablesNames = self.networkScanner.getHostnames()
        self.actionLabelText = ''
        self.moveStartLoadLabelUp(0.95)
        self.add_widget(self.scrollView)
        self.innerLayout.createFromScan()
        self.scrollView.add_widget(self.innerLayout)
        self.addButtons()

    # Method called when button 'Load Configuration' is pressed.
    def loadYaml(self):
        self.removeWidgets()
        self.moveStartLoadLabelUp(0.95)
        self.add_widget(self.scrollView)
        self.innerLayout = self.yamlParser.openLoadYamlPopup()
        self.scrollView.add_widget(self.innerLayout)
        self.addButtons()

    # Move buttons 'Scan' and 'Load Configuration' on top of the screen.
    def moveStartLoadLabelUp(self, yPos):
        self.scanButtonPosY = yPos
        self.loadButtonPosY = yPos
        self.actionLabelPosY = yPos

    # Add additional four buttons on the bottom of the screen.
    def addButtons(self):
        addTableButton = AddButton('addTableButton', 'Add Table', (0.1, 0.05), (0.2, 0.1), (0, 0))
        addTableButton.bind(on_press = self.innerLayout.openTableNamePopup)
        removeTableButton = AddButton('removeTableButton', 'Remove Selected\n      Tables', (0.3, 0.05), (0.2, 0.1), (0, 0))
        removeTableButton.bind(on_press = self.innerLayout.removeTable)
        createYamlButton = AddButton('createYamlButton', 'Create Yaml File', (0.7, 0.05), (0.2, 0.1), (0, 0))
        createYamlButton.bind(on_press = self.yamlParser.openSaveYamlPopup)
        visualizeRvizButton = AddButton('visualizeRvizButton', 'Rviz', (0.9, 0.05), (0.2, 0.1), (0, 0))
        visualizeRvizButton.bind(on_press = self.rvizVizualization.visualizeTables)
        self.add_widget(addTableButton)
        self.add_widget(removeTableButton)
        self.add_widget(createYamlButton)
        self.add_widget(visualizeRvizButton)

    # Remove all widgets except buttons.
    def removeWidgets(self):
        self.remove_widget(self.scrollView)
        self.scrollView.remove_widget(self.innerLayout)
        self.innerLayout.removeInnerWidgets()


# Main application class.
class GuiApp(App):
    def build(self):
        return Controller()
        


if __name__ == '__main__':
    rospy.init_node('visualization_gui_node')
    GuiApp().run()