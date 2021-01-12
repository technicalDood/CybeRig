import pymel.core as pm

def addBoundManagerNode(name='Default'):
    '''
    create a manager node to link CR objects
    '''
    manager = pm.group(empty=True,name=('MNG_BOUND_'+name))
    manager.addAttr('boundMng')
    for x in range(0,10):
        #10 slots for 10 drivers
        #anymore than that is already nonsense
        an = 'Manager'+str(x)
        manager.addAttr(an)
    manager.hiddenInOutliner.set(True)
    return manager

def addDriverManagerNode(name='Default'):
    '''
    create a driver manager node to link CR objects
    '''
    manager = pm.group(empty=True,name=('MNG_DRIVER_'+name))
    manager.addAttr('drvMng')
    manager.addAttr('Manager')
    manager.hiddenInOutliner.set(True)
    return manager

def addConnectorManagerNode(name='Default'):
    '''
    create a Connector manager node to link CR objects
    '''
    manager = pm.group(empty=True,name=('MNG_CONNECTOR_'+name))
    manager.addAttr('cntMng')
    manager.addAttr('Manager')
    manager.hiddenInOutliner.set(True)
    return manager

#-------------------------
def getJntsFromBoundManager(manager):
    '''
    return the jointList from bound manager
    the joints chain must be single chain, if not prompt error
    '''
    if manager.name().startswith('MNG_BOUND_'):
        jntList = manager.boundMng.outputs()
        jntList = reorderSingleChainJointList(jntList)
        if isSingleChain(jntList):
            return jntList #check later if this is single chain
        else:
            raise AttributeError('this bound manager node does not connect to a proper joint chain, please consider remove and rebuild!')
    else:
        raise AttributeError('This is not a bound manager!')

def getEmptyDriverManagerSlot(manager):
    '''
    return the first empty slot of driver manager to hook up with driver
    '''
    for x in range(0,10):
        #10 slots for 10 drivers
        #anymore than that is already nonsense
        an = 'Manager'+str(x)
        at = pm.PyNode(manager.name()+'.'+an)
        if len(at.inputs()) == 0:
            return at
    return None

def clearBoundManager(manager):
    jntList = getJntsFromBoundManager(manager)
    for j in jntList:
        j.bound.disconecct()

def cleanupBoundManagers():
    '''
    Remove all empty boundManagers
    '''
    for x in pm.ls('MNG_BOUND_*'):
        if (x.type()=='transform') and (len(x.boundMng.outputs()) == 0):
            print('Remove {}'.format(x))
            pm.delete(x)

def getJntsFromDriverManager(manager):
    '''
    return the jointList from driver manager
    the joints chain must be single chain, if not prompt error
    '''
    if manager.name().startswith('MNG_DRIVER_'):
        jntList = manager.drvMng.outputs()
        return jntList #check later if this is single chain

def clearDriverManager(manager):
    jntList = getJntsFromDriverManager(manager)
    for j in jntList:
        j.driver.disconecct()

def cleanupDriverManagers():
    '''
    Remove all empty driverManagers
    '''
    for x in pm.ls('MNG_DRIVER_*'):
        if (x.type()=='transform') and (len(x.drvMng.outputs()) == 0):
            print('Remove {}'.format(x))
            pm.delete(x)

def getIOFromDefaultConnector(manager):
    '''
    return the jointList from driver manager
    '''
    if manager.name().startswith('MNG_CONNECTOR_'):
        connectorNodes = manager.cntMng.outputs()
        connectorOutputSet = []
        for connectorNode in connectorNodes:
            connectorAttrs = [a for a in connectorNode.listAttr(userDefined=True) if a.attrName().startswith('c_')]
            drvOutputs = [oa.inputs(plugs=True)[0] for oa in connectorAttrs]
            bnInputs = [ia.outputs(plugs=True)[0] for ia in connectorAttrs]
            connectorOutputSet.append([drvOutputs,bnInputs])
        return connectorOutputSet

#-------------------------
def reorderSingleChainJointList(jointList):
    '''
    reorder the joint list from top to bottom of joints chain
    raise error if this chain is not possible to be re-ordered
    '''
    #find bottom joint
    reorderList = []
    btmJnt = None
    for j in jointList:
        isBtm = True
        children = j.getChildren()
        for c in children:
            if c in jointList:
                isBtm = False
                break

        if isBtm:
            btmJnt = j
            reorderList.append(btmJnt)
            break

    while len(reorderList) < len(jointList):
        nextJnt = btmJnt.getParent()
        if nextJnt in jointList:
            btmJnt = nextJnt
            reorderList.append(btmJnt)
        else:
            raise AttributeError('This is not a single joint chain, unable to reorder')
        
    reorderList.reverse()
    return reorderList


def isSingleChain(jointList):
    '''
    check if the joint list is indeed a single joint chain
    '''
    for x in range(len(jointList)-1,0,-1):
        c = jointList[x]
        p = jointList[x-1]
        if c.type() != 'joint':
            return False
        if c.getParent() != p:
            return False
    
    return True

def duplicateSingleChain(jointList):
    '''
    true duplication of a joint list
    must be single chain
    the dup chain will be outside of any hierachy
    '''
    if isSingleChain(jointList):
        dupJointList = []
        pm.select(cl=True)
        for j in jointList:
            dj = pm.joint(n='dup_'+j.name())
            alignTransform(j,dj)
            dupJointList.append(dj)

        return dupJointList
    else:
        raise Exception('input joint list is NOT a single chain')



# --------------------------------------------------------------
# RIGGING UTILS
# --------------------------------------------------------------
CONTROLLERSTYPE = ["Default","circle","square","sphere","cube","arrow","cross","diamond"]
def shapeGenerate(shape="circle",name="new_controller"):
	'''
	generate controller shape
	general rule is 1 controller fit 15 unit tall figure
	'''
	if shape == "circle":
		c = pm.circle(nr=[1,0,0],ch=False,n=name)[0]
	elif shape == "square":
		c = pm.curve(d=1, p=[(-1,0,1), (-1,0,-1), (1,0,-1), (1,0,1), (-1,0,1)], k=[0,1,2,3,4],n=name )
	elif shape == "sphere":
		c = pm.curve(d=1, p=[(-1,0,0),(-0.66,0,-0.66),(0,0,-1),(0.66,0,-0.66),(1,0,0),(0.66,0,0.66),(0,0,1),(-0.66,0,0.66),(-1,0,0),(-0.66,0.66,0),(0,1,0),(0.66,0.66,0),(1,0,0),(0.66,-0.66,0),(0,-1,0),(-0.66,-0.66,0),(-1,0,0),(-0.66,0.66,0),(0,1,0),(0,0.66,0.66),(0,0,1),(0,-0.66,0.66),(0,-1,0),(0,-0.66,-0.66),(0,0,-1),(0,0.66,-0.66),(0,1,0)], 
								k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26],n=name  )
	elif shape == "cube":
		c= pm.curve(d=1, p=[(1,-1,1),(1,-1,-1),(-1,-1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(1,1,-1),(-1,1,-1),(-1,1,1),(1,1,1),(-1,1,1),(-1,-1,1),(-1,-1,-1),(-1,1,-1),(1,1,-1),(1,-1,-1)], 
								k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15],n=name  )
	elif shape == "arrow":
		c= pm.curve(d=1, p=[(-0.5,0,-1),(-0.5,0,0.25),(-1,0,0.25),(0,0,1),(1,0,0.25),(0.5,0,0.25),(0.5,0,-1),(-0.5,0,-1)], 
								k=[0,1,2,3,4,5,6,7],n=name )
	elif shape == "cross":
		c = pm.curve(d=1, p=[(-0.33333,0,1),(-0.33333,0,0.333333),(-1,0,0.333333),(-1,0,-0.333333),(-0.33333,0,-0.333333),(-0.33333,0,-1),(0.333333,0,-1),(0.333333,0,-0.333333),(1,0,-0.333333),(1,0,0.333333),(0.333333,0,0.333333),(0.333333,0,1),(-0.33333,0,1)], 
								k=[0,1,2,3,4,5,6,7,8,9,10,11,12],n=name )
	elif shape == "diamond":
		c = pm.curve(d=1, p=[(0,0,1),(-1,0,0),(0,0,-1),(1,0,0),(0,0,1),(0,1,0),(0,0,-1),(0,-1,0),(0,0,1),(1,0,0),(0,1,0),(-1,0,0),(0,-1,0),(1,0,0)], 
								k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13],n=name )
	else:
		c=None
	return c

def matchCustomized(base,target):
    baseShape = base.getShapes()[0]
    targetShape = target.getShapes()[0]
    
    baseBBSize = (base.getBoundingBox().max() - base.getBoundingBox().min()).length()
    targetBBSize = (target.getBoundingBox().max() - target.getBoundingBox().min()).length()
    scaleRatio = baseBBSize/targetBBSize
    pm.select(targetShape.cv,r=True)
    pm.scale(scaleRatio,scaleRatio,scaleRatio,r=True,ocp=True)
    pm.select(cl=True)
    
    #
    baseLW = baseShape.lineWidth.get()
    targetShape.lineWidth.set(baseLW)
    #
    baseColorSet = baseShape.overrideEnabled.get()
    if baseColorSet:
        targetShape.overrideEnabled.set(True)
        targetShape.overrideColor.set(baseShape.overrideColor.get())

def alignTransform(base,target):
	pm.delete(pm.parentConstraint(base,target))

def isSamePlane(jntList):
    return True

def snapOnPlane(jntList):
    '''
    using the first 3 joints in list to align everybody else
    '''
    pass

def jointOrient(jntList, **kwargs):
    pass