#execfile('path/xlink.py')
from abaqus import *
from abaqusConstants import *
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
import regionToolset
import mesh
import os

#**********************************************************************************************************

# Create a model with the name of tetrahedron
NameOfModel='xLink_tens'
tmpModel = mdb.Model(name='tmp')
if mdb.models.has_key('Model-1'):
    del mdb.models['Model-1']
if mdb.models.has_key(NameOfModel):
    del mdb.models[NameOfModel]
myModel = mdb.Model(name=NameOfModel)
del mdb.models['tmp']


#Parameters
#Fiber radius/Diameter, Length
R_Fiber=1.0
# aspectRatio = 10
#L_Fiber_Unit = 2*aspectRatio*R_Fiber
L_Fiber_Unit = 40
Angle_Fiber_D=45 #Degree
Angle_Fiber_R=Angle_Fiber_D*pi/180 #Radian
Distance_Fiber_R=L_Fiber_Unit/2.0
#Number of Unit: 
N_Unit=5
Distance_Fiber_Vertical=R_Fiber*0.1

#COF (Coefficient of Friction)
COF=0.15
#Meshing Factor
mesh_factor=1.5

# Loading displacement in z direction
zLoading = 0.2*(L_Fiber_Unit*(N_Unit-1.0/2.0)/sin(Angle_Fiber_R)+
                                 2*R_Fiber*cos(Angle_Fiber_R)+R_Fiber)

#Define Materials
#FIBER
Density_Fiber = 2540.0
YoungModules_Fiber = 78.9E+9
Poisson_Fiber = 0.183
myMaterial_1 = myModel.Material(name='Fiber')
myMaterial_1.Density(table=((Density_Fiber, ), ))
myMaterial_1.Elastic(table=((YoungModules_Fiber, Poisson_Fiber), ))
S_Fiber = myModel.HomogeneousSolidSection(name='S_Fiber',material='Fiber', thickness=None)

#Binder
Density_Binder = 1000.0
YoungModules_Binder = 1.827E+9
Poisson_Binder = 0.35
myMaterial_2 = myModel.Material(name='Binder')
myMaterial_2.Density(table=((Density_Binder, ), ))
myMaterial_2.Elastic(table=((YoungModules_Binder, Poisson_Binder), ))
S_Binder = myModel.HomogeneousSolidSection(name='S_Binder',material='Binder', thickness=None)


myModel.ContactProperty('noFric')
myModel.interactionProperties['noFric'].TangentialBehavior(
    formulation=FRICTIONLESS)
#: The interaction property "noFric" has been created.
myModel.ContactProperty('hardContact')
#myModel.interactionProperties['hardContact'].TangentialBehavior(
#    formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, 
#    pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, table=((
#    COF, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, 
#    fraction=0.005, elasticSlipStiffness=None)
myModel.interactionProperties['hardContact'].NormalBehavior(
    pressureOverclosure=LINEAR, contactStiffness=1000000000000.0, 
    constraintEnforcementMethod=DEFAULT)
#: The interaction property "hardContact" has been created.


#**********************************************************************************************************
#Create the Geometries
mySketch_1 = myModel.ConstrainedSketch(name='FiberProfile', sheetSize=L_Fiber_Unit)

Point_center=(0.0,0.0)
Point_1=(R_Fiber,0)
mySketch_1.CircleByCenterPerimeter(center=Point_center, point1=Point_1)
myPart_1 = myModel.Part(name='Fiber', dimensionality=THREE_D,type=DEFORMABLE_BODY)
myPart_1.BaseSolidExtrude(sketch = mySketch_1, depth=L_Fiber_Unit*N_Unit)

#Meshing the Fiber
#Global Element Size
myPart_1.seedPart(size=L_Fiber_Unit/10.0/mesh_factor, deviationFactor=0.1)

elemType1 = mesh.ElemType(elemCode=C3D8I,
                          elemLibrary=STANDARD,
                          secondOrderAccuracy=OFF,
                          distortionControl=DEFAULT)
elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=STANDARD)
elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=STANDARD)

cell_region = regionToolset.Region(cells=myPart_1.cells)
myPart_1.SectionAssignment(region=cell_region, sectionName='S_Fiber',
                           offset=0.0,offsetType=MIDDLE_SURFACE,
                           offsetField='',
                           thicknessAssignment=FROM_SECTION)
myPart_1.setElementType(regions=cell_region, elemTypes=(elemType1, ))

Center_1=(0.0,0.0,0.0)
Center_2=(0.0,0.0,L_Fiber_Unit*N_Unit)
Center_3=(0.0,R_Fiber,L_Fiber_Unit*N_Unit/2.0)

face_1=myPart_1.faces.findAt((Center_1,),)
face_2=myPart_1.faces.findAt((Center_3,),)
#face_2=myPart_1.faces.findAt(Center_2)
geometrySourceSides=regionToolset.Region(faces=face_1)
geomConnectingSides=regionToolset.Region(faces=face_2)
myPart_1.generateBottomUpSweptMesh(cell=myPart_1.cells[0],
                                   geometrySourceSide=geometrySourceSides,
                                   geometryConnectingSides=geomConnectingSides)

#***************************************************************************
#Assembling the Fibers

myAssembly = myModel.rootAssembly
myAssembly.DatumCsysByDefault(CARTESIAN)
myInstance_1 = myAssembly.Instance(name='Fiber', part=myPart_1, dependent=ON)

#Create the fiber pattern and rotate to the positions
myAssembly.LinearInstancePattern(instanceList=('Fiber', ),
                                 direction1=(1.0, 0.0,0.0),
                                 direction2=(0.0, 1.0, 0.0),
                                 number1=N_Unit, number2=2, 
                                 spacing1=Distance_Fiber_R*2.0,
                                 spacing2=2.0*R_Fiber+Distance_Fiber_Vertical)

myAssembly.features.changeKey(fromName = 'Fiber', toName = 'Fiber-lin-1-1')

spacing_1 = Distance_Fiber_R*2
spacing_2 = 2.0*R_Fiber
axisDir_x = (1.0,0.0,0.0)
axisDir_y = (0.0,1.0,0.0)
axisDir_z = (0.0,0.0,1.0)

##
Vector=(-L_Fiber_Unit*(N_Unit-1)/2.0,-R_Fiber-Distance_Fiber_Vertical/2.0,-L_Fiber_Unit*N_Unit/2.0)
Fiber_Instances = []
for ii in range(1,1+N_Unit):
    for jj in range(1,3):
        instance_name = 'Fiber-lin-'+str(ii)+'-'+str(jj)
        fCrack_name = 'fCrack-lin-'+str(ii)+'-'+str(jj)
        if jj ==1:
            myAssembly.translate(instanceList=(instance_name, ),vector=Vector)
            myAssembly.rotate(instanceList=(instance_name, ),
                              axisPoint = (0.0, 0.0, 0.0),
                              axisDirection=axisDir_y, angle=Angle_Fiber_D)

        if jj ==2:
            myAssembly.translate(instanceList=(instance_name, ),vector=Vector)
            myAssembly.rotate(instanceList=(instance_name, ),
                              axisPoint = (0.0, 0.0, 0.0),
                              axisDirection=axisDir_y, angle=-Angle_Fiber_D)
        Fiber_Instances.append(myAssembly.instances[instance_name])
	
##
#Part Binder
Ratio_Binder_Fiber=2.8        
mySketch_2 = myModel.ConstrainedSketch(name='BinderProfile', sheetSize=L_Fiber_Unit)

Point_center=(0.0,0.0)
Point_1=(0.0,R_Fiber*Ratio_Binder_Fiber)
Point_2=(0.0,-R_Fiber*Ratio_Binder_Fiber)
mySketch_2.Line(point1=Point_1, point2=Point_2)
mySketch_2.ArcByCenterEnds(center=Point_center, point1=Point_1,
                           point2=Point_2, direction=CLOCKWISE)
mySketch_2.ConstructionLine(point1=Point_1, point2=Point_2)
myPart_2 = myModel.Part(name='Binder', dimensionality=THREE_D,type=DEFORMABLE_BODY)
myPart_2.BaseSolidRevolve(sketch=mySketch_2, angle=360.0, flipRevolveDirection=OFF)

myInstance_2 = myAssembly.Instance(name='Binder', part=myPart_2, dependent=ON)

#Create the holes in the binder
mySketch_3 = myModel.ConstrainedSketch(name='HoleProfile', sheetSize=L_Fiber_Unit)

Point_center=(0.0,0.0)
Point_1=(R_Fiber,0)
mySketch_3.CircleByCenterPerimeter(center=Point_center, point1=Point_1)
myPart_3 = myModel.Part(name='Hole', dimensionality=THREE_D,type=DEFORMABLE_BODY)
myPart_3.BaseSolidExtrude(sketch = mySketch_3, depth=R_Fiber*Ratio_Binder_Fiber*2.0)

#***************************************************************************
#Assembling
myInstance_3 = myAssembly.Instance(name='Hole', part=myPart_3, dependent=ON)
##
###Create the tetrahedra pattern and rotate to the positions
myAssembly.LinearInstancePattern(instanceList=('Hole', ),
                                 direction1=(1.0, 0.0,0.0),
                                 direction2=(0.0, 1.0, 0.0),
                                 number1=1, number2=2, 
                                 spacing1=Distance_Fiber_R*2.0,
                                 spacing2=2.0*R_Fiber+Distance_Fiber_Vertical)

Vector=(0.0,-R_Fiber-Distance_Fiber_Vertical/2.0,-R_Fiber*Ratio_Binder_Fiber)

myAssembly.features.changeKey(fromName = 'Hole', toName = 'Hole-lin-1-1')
myAssembly.translate(instanceList=('Hole-lin-1-1',),vector=Vector)
myAssembly.translate(instanceList=('Hole-lin-1-2',),vector=Vector)

myAssembly.rotate(instanceList=('Hole-lin-1-1',), axisPoint = (0.0, 0.0, 0.0),
                  axisDirection=axisDir_y, angle=Angle_Fiber_D)
myAssembly.rotate(instanceList=('Hole-lin-1-2',), axisPoint = (0.0, 0.0, 0.0),
                  axisDirection=axisDir_y, angle=-Angle_Fiber_D)

myAssembly.InstanceFromBooleanCut(name='Part-1',
                                  instanceToBeCut=myAssembly.instances['Binder'],
                                  cuttingInstances=(myAssembly.instances['Hole-lin-1-1'],
                                                    myAssembly.instances['Hole-lin-1-2'], ),
                                  originalInstances=DELETE)
del myModel.parts['Binder']
del myModel.parts['Hole']

myModel.parts.changeKey(fromName='Part-1', toName='Binder')
myAssembly.regenerate()
myAssembly.features.changeKey(fromName='Part-1-1', toName='Binder')
myInstance_2 = myAssembly.instances['Binder']

myPart_2 = myModel.parts['Binder']
myPart_2.seedPart(size=R_Fiber*Ratio_Binder_Fiber/2.0/mesh_factor, deviationFactor=0.1)
cell_region = regionToolset.Region(cells=myPart_2.cells)
myPart_2.SectionAssignment(region=cell_region, sectionName='S_Binder',
                           offset=0.0,offsetType=MIDDLE_SURFACE,
                           offsetField='',
                           thicknessAssignment=FROM_SECTION)
myPart_2.setMeshControls(regions=myPart_2.cells, elemShape=TET, technique=FREE)
myPart_2.generateMesh()


Vector=(0.0,0.0,-spacing_1*(N_Unit-1)/2.0/sin(Angle_Fiber_R))
myAssembly.translate(instanceList=('Binder',),vector=Vector)

myAssembly.LinearInstancePattern(instanceList=('Binder', ),
                                 direction1=(1.0, 0.0,0.0),
                                 direction2=(0.0, 0.0, 1.0),
                                 number1=N_Unit, number2=N_Unit, 
                                 spacing1=Distance_Fiber_R*2,
                                 spacing2=Distance_Fiber_R*2)
myAssembly.features.changeKey(fromName='Binder', toName='Binder-lin-1-1')
spacing_1 = Distance_Fiber_R*2
spacing_2 = 2.0*R_Fiber
axisDir_x = (1.0,0.0,0.0)
axisDir_y = (0.0,1.0,0.0)
axisDir_z = (0.0,0.0,1.0)

for ii in range(1,1+N_Unit):
    for jj in range(1,1+N_Unit):
        instance_name = 'Binder-lin-'+str(ii)+'-'+str(jj)
        bCrack_name    = 'bCrack-lin-'+str(ii)+'-'+str(jj)
        x0=(ii-1)*Distance_Fiber_R*2
        z0=(jj-1)*Distance_Fiber_R*2
        x1=x0*cos(-Angle_Fiber_R)+z0*sin(-Angle_Fiber_R)
        z1=-x0*sin(-Angle_Fiber_R)+z0*cos(-Angle_Fiber_R)
        x=x1-x0
        z=z1-z0
        Vector=(x,0.0,z)
        myAssembly.translate(instanceList=(instance_name,),vector=Vector)

#Tie constraint
myModel.contactDetection(name='Tie_Constraints',defaultType=TIE,
                         separationTolerance=Distance_Fiber_Vertical/5.0) 

#Create Plate
mySketch_3 = myModel.ConstrainedSketch(name='Plate_profile',
                                       sheetSize=L_Fiber_Unit*N_Unit)

Point_1=(-L_Fiber_Unit*N_Unit/sin(Angle_Fiber_R)/2.0,4*R_Fiber*Ratio_Binder_Fiber)
Point_2=(L_Fiber_Unit*N_Unit/sin(Angle_Fiber_R)/2.0,-4*R_Fiber*Ratio_Binder_Fiber)

mySketch_3.rectangle(point1=Point_1, point2=Point_2)
myPart_3 = myModel.Part(name='Plate', dimensionality=THREE_D,
                        type=DEFORMABLE_BODY)
myPart_3.BaseSolidExtrude(sketch=mySketch_3, depth=R_Fiber)
myRP_Plate = myPart_3.ReferencePoint(point=(0,0,0))
RP_Plate_ID = myRP_Plate.id
p = myModel.parts['Plate']
r = p.referencePoints
refPoints=(r[2], )
p.Set(referencePoints=refPoints, name='RP')

region = regionToolset.Region(cells=myPart_3.cells)
myPart_3.SectionAssignment(region=region, sectionName='S_Fiber',
                           offset=0.0,offsetType=MIDDLE_SURFACE,
                           offsetField='',thicknessAssignment=FROM_SECTION)
myPart_3.seedPart(size=R_Fiber, deviationFactor=0.1, minSizeFactor=0.1)
region = regionToolset.Region(cells=myPart_3.cells)
myPart_3.setElementType(regions=region, elemTypes=(elemType1,))
myPart_3.setMeshControls(regions=myPart_3.cells, technique=SWEEP,
                         algorithm=ADVANCING_FRONT)
myPart_3.generateMesh()

myInstance_3 = myAssembly.Instance(name='Plate', part=myPart_3, dependent=ON)
Vector=(0.0,0.0,-L_Fiber_Unit*(N_Unit-1.0/2.0)/sin(Angle_Fiber_R)/2.0-R_Fiber*cos(Angle_Fiber_R)-R_Fiber)
myAssembly.translate(instanceList=('Plate',),vector=Vector)

myAssembly.LinearInstancePattern(instanceList=('Plate', ),
                                 direction1=(1.0, 0.0,0.0),
                                 direction2=(0.0, 0.0, 1.0),
                                 number1=1, number2=2, 
                                 spacing1=0,
                                 spacing2=L_Fiber_Unit*(N_Unit-1.0/2.0)/sin(Angle_Fiber_R)+
                                 2*R_Fiber*cos(Angle_Fiber_R)+R_Fiber)
myAssembly.features.changeKey(fromName = 'Plate', toName = 'Plate-lin-1-1')

#Rigid Body Setting
region2=regionToolset.Region(cells=myAssembly.instances['Plate-lin-1-1'].cells[0:1])
region1=regionToolset.Region(referencePoints=(myAssembly.instances['Plate-lin-1-1'].
                                              referencePoints[2],) )
myModel.RigidBody(name='Plate-lin-1-1',refPointRegion=region1,bodyRegion=region2)

region2=regionToolset.Region(cells=myAssembly.instances['Plate-lin-1-2'].cells[0:1])
region1=regionToolset.Region(referencePoints=(myAssembly.instances['Plate-lin-1-2'].
                                              referencePoints[2],) )
myModel.RigidBody(name='Plate-lin-1-2',refPointRegion=region1,bodyRegion=region2)

myModel.StaticStep(name='Step-1', previous='Initial', 
    maxNumInc=1000, initialInc=0.1, minInc=1e-08, maxInc=0.1, nlgeom=ON)
myModel.steps['Step-1'].setValues(stabilizationMagnitude=0.0002, 
    stabilizationMethod=DISSIPATED_ENERGY_FRACTION, 
    continueDampingFactors=False, adaptiveDampingRatio=None)
myModel.steps['Step-1'].control.setValues(allowPropagation=OFF, 
    resetDefaultValues=OFF, timeIncrementation=(4.0, 8.0, 9.0, 16.0, 10.0, 4.0, 
    12.0, 10.0, 6.0, 3.0, 50.0))
#: Step definition

myModel.ContactStd(name='contactDef', createStepName='Initial')
myModel.interactions['contactDef'].includedPairs.setValuesInStep(
    stepName='Initial', useAllstar=ON)
myModel.interactions['contactDef'].contactPropertyAssignments.appendInStep(
    stepName='Initial', assignments=((GLOBAL, SELF, 'hardContact'), ))
#: General contact has been created.

#s1 = myAssembly.instances['Plate-lin-1-1'].faces
#region1=regionToolset.Region(side1Faces=s1)
#constraintPartName1 = 'Fiber-lin-'+str(N_Unit)+'-1'
#f1 = myAssembly.instances[constraintPartName1].faces
#f2 = myAssembly.instances['Fiber-lin-1-2'].faces
#region2=regionToolset.Region(faces=f1+f2)
#mdb.models['xLink'].Tie(name='constraint-fiber-platen-1', master=region1, slave=region2, 
#    positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)
#s1 = myAssembly.instances['Plate-lin-1-2'].faces
#region1=regionToolset.Region(side1Faces=s1)
#constraintPartName1 = 'Fiber-lin-'+str(N_Unit)+'-2'
#f1 = myAssembly.instances[constraintPartName1].faces
#f2 = myAssembly.instances['Fiber-lin-1-1'].faces
#region2=regionToolset.Region(faces=f1+f2)
#mdb.models['xLink'].Tie(name='constraint-fiber-platen-2', master=region1, slave=region2, 
#    positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)
# Constraints between fiber and plate have been created to help convergence problem.

r1 = myAssembly.instances['Plate-lin-1-2'].referencePoints
refPoints1=(r1[2], )
region = regionToolset.Region(referencePoints=refPoints1)
myModel.DisplacementBC(name='BC-1', createStepName='Step-1', 
    region=region, u1=0.0, u2=0.0, u3=-zLoading, ur1=0.0, ur2=0.0, ur3=0.0, 
    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
    localCsys=None)
r1 = myAssembly.instances['Plate-lin-1-1'].referencePoints
refPoints1=(r1[2], )
region = regionToolset.Region(referencePoints=refPoints1)
myModel.EncastreBC(name='BC-2', createStepName='Step-1', 
    region=region, localCsys=None)
# Boundary conditions have been created.

myModel.FieldOutputRequest(name='F-Output-2', 
    createStepName='Step-1', variables=('CSTATUS', 'PSILSM', 'STATUSXFEM'))
# Field output includes contact and failure index
regionDef1=myAssembly.allInstances['Plate-lin-1-1'].sets['RP']
mdb.models['xLink'].HistoryOutputRequest(name='H-Output-2', 
    createStepName='Step-1', variables=('U3', 'RF3'), region=regionDef1, 
    sectionPoints=DEFAULT, rebar=EXCLUDE)
regionDef2=myAssembly.allInstances['Plate-lin-1-2'].sets['RP']
mdb.models['xLink'].HistoryOutputRequest(name='H-Output-3', 
    createStepName='Step-1', variables=('U3', 'RF3'), region=regionDef2, 
    sectionPoints=DEFAULT, rebar=EXCLUDE)
# History output requests have been created.

jobName = "xLink-elastic-f"+str(N_Unit)
mdb.Job(name=jobName, model=NameOfModel, description='', type=ANALYSIS, 
    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
    scratch='', multiprocessingMode=DEFAULT, numCpus=8, numDomains=8, 
    numGPUs=0)
# Job has been been created.
# mdb.jobs[jobName].submit(consistencyChecking=OFF)

mdb.saveAs(pathName=('xLink-elastic-f'+str(N_Unit)+'.cae'))
#: The model database has been saved to "C:\Temp\Crosslink\xLink-fN_Unit.cae".

session.viewports['Viewport: 1'].setValues(displayedObject=myAssembly)
session.viewports['Viewport: 1'].view.setValues(session.views['Bottom'])
#: Display the assembly
