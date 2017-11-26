#Author-Autodesk Inc.
#Description-Caculate the intersections between the selected curve/surface/body/component/occurrence and curve/surface.
# non planar surface does not support for now

import adsk.core, adsk.fusion, traceback
import os
import tempfile
import subprocess

# global set of event handlers to keep them referenced for the duration of the command
handlers = []
app = adsk.core.Application.get()
if app:
    ui = app.userInterface

RADIOBUTTONGROUP = adsk.core.RadioButtonGroupCommandInput.cast(None)
resourceDir = ""

class MeshBooleanCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            command = args.firingEvent.sender
            inputs = command.commandInputs

            tmpdir = "c:/t" ## tempfile.mkdtemp()
            files = []
            for i in [0,1]:
                mesh = inputs[i].selection(0).entity.mesh;
                filename = os.path.join(tmpdir, str(i) + '.obj')
                files += [filename]
                f = open(filename, 'w')
                for pt in mesh.nodeCoordinates:
                    f.write("v {0} {1} {2}\n".format(pt.x, pt.y, pt.z))
                j = 3
                for ind in mesh.triangleNodeIndices:
                  if j == 3:
                      f.write("\nf")
                      j = 0
                  f.write(" {0}".format(ind + 1))
                  j += 1

                f.close()

            exe = "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/out/MeshBooleanApp/Release/MeshBooleanApp.exe"
            exed = "C:/Users/marco\OneDrive/Documents/PROJECTS/polytriagnulation/out/MeshBooleanApp/Debug/MeshBooleanApp.exe"

            operation = RADIOBUTTONGROUP.selectedItem.name
            curent_dir = os.getcwd()
            os.chdir(tmpdir)
            ##msdev = "C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/Common7/IDE/devenv.exe"
            ##subprocess.check_call([msdev, exe, files[0], files[1], operation])
            subprocess.check_call([exed, files[0], files[1], operation])
            activeDoc = adsk.core.Application.get().activeDocument
            design = activeDoc.design
            rootComp = design.rootComponent 
            rootComp.meshBodies.add(tmpdir + "/result.obj", 0)
            os.chdir(curent_dir)

        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class MeshBooleanCommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            # when the command is done, terminate the script
            # this will release all globals which will remove all event handlers
            adsk.terminate()
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class MeshBooleanValidateInputHandler(adsk.core.ValidateInputsEventHandler):
    def __init__(self):
        super().__init__()
       
    def notify(self, args):
        try:
            sels = ui.activeSelections;
            if len(sels) == 2:
                args.areInputsValid = True
            else:
                args.areInputsValid = False
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class MeshBooleanCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            cmd = args.command

            onExecute = MeshBooleanCommandExecuteHandler()
            cmd.execute.add(onExecute)
            handlers.append(onExecute)

            onDestroy = MeshBooleanCommandDestroyHandler()
            cmd.destroy.add(onDestroy)
            handlers.append(onDestroy)

            onValidateInput = MeshBooleanValidateInputHandler()
            cmd.validateInputs.add(onValidateInput)
            handlers.append(onValidateInput)

            # keep the handler referenced beyond this function
            #define the inputs
            inputs = cmd.commandInputs
            i1 = inputs.addSelectionInput('entity', 'Entity One', 'Please select a mesh')
            i1.addSelectionFilter(adsk.core.SelectionCommandInput.MeshBodies);
            i1.setSelectionLimits(0, 1)
            i2 = inputs.addSelectionInput('entity', 'Entity Two', 'Please select a mesh')
            i2.addSelectionFilter(adsk.core.SelectionCommandInput.MeshBodies);
            i2.setSelectionLimits(0, 1)

            # Create radio button group input.
            global RADIOBUTTONGROUP
            RADIOBUTTONGROUP = inputs.addRadioButtonGroupCommandInput('BoolOperation', 'Operation button group')
            radioButtonItems = RADIOBUTTONGROUP.listItems
            values = [ "UNION", "INTERSECTION", "DIFFERENCE", 
                       "SPLIT", "SPLITA", "SPLITB", "A_IN_B", "B_IN_A",
                       "A_OUT_B", "B_OUT_A", "A_OVERLAP", "B_OVERLAP" ]
            for v in values:
                radioButtonItems.add(v, v == "UNION")
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def run(context):
    try:
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        if not design:
            ui.messageBox('It is not supported in current workspace, please change to MODEL workspace and try again.')
            return
        commandDefinitions = ui.commandDefinitions
        # check the command exists or not
        command_name = 'MeshBooleanDef'
        cmdDef = commandDefinitions.itemById(command_name)
        global resourceDir
        if not cmdDef:
            resourceDir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'resources') # absolute resource file path is specified
            cmdDef = commandDefinitions.addButtonDefinition(command_name,
                    'MeshBoolean',
                    'Boolean operation on meshes',
                    resourceDir)

        onCommandCreated = MeshBooleanCommandCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        # keep the handler referenced beyond this function
        handlers.append(onCommandCreated)
        inputs = adsk.core.NamedValues.create()
        cmdDef.execute(inputs)

        # prevent this module from being terminate when the script returns, because we are waiting for event handlers to fire
        adsk.autoTerminate(False)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
