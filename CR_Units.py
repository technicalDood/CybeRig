import pymel.core as pm

from .CR_Utils import *

class BoundJoints(object):
    '''
    This base class handles linear single chain bound joints
    '''
    def __init__(self,jointList=[],name='Default',manager=None):
        self.iname = name

        self.jointList = jointList

        if len(self.jointList) > 0:
            self.startJnt = self.jointList[0]
            self.endJnt = self.jointList[-1]
            self.parent = self.startJnt.getParent()
        else:
            self.startJnt = None
            self.endJnt = None
            self.parent = None

        if manager is None:
            self.manager = self._createManager(self.iname) #create fresh manager
            self.iname = self.manager.name().replace('MNG_BOUND_','')
        else:
            self.manager = manager

        self.jointCount = self.getJointsCount()
        self.connectorPlugs = []
        self.skinNodeList = []
        self.constructSkinList()
        #create manager

    @classmethod
    def from_values(cls,*args):
        return cls(args)

    @classmethod
    def from_manager(cls,manager):
        jntList = getJntsFromBoundManager(manager)
        name = manager.name().replace('MNG_BOUND_','')
        return cls(jntList,name,manager)

    # private methods
    def _createManager(self,name):
        manager = addBoundManagerNode(name)
        print('start to connect')
        for j in self.jointList:
            if not j.hasAttr('bound'):
                j.addAttr('bound')
            print('connect {} to {}'.format(manager,j))
            manager.boundMng >> j.bound
        return manager

    def _setManager(self,manager):
        #clear manager first
        clearBoundManager(manager)
        for j in self.jointList:
            if not j.hasAttr('bound'):
                j.addAttr('bound')
            manager.boundMng >> j.bound
        self.manager = manager
        self.iname = self.manager.name().replace('MNG_BOUND_','')

    #construct method
    def constructSkinList(self):
        '''
        listing all the skin node link to this joint chain
        '''
        self.skinNodeList = []
        for j in self.jointList:
            skinNodes = [s for s in j.worldMatrix[0].outputs() if s.type() == 'skinCluster']
            for sn in skinNodes:
                self.skinNodeList.append(sn)
            self.skinNodeList = list(set(self.skinNodeList))

    #create methods
    def createDefaultDriver(self,attributeList,suffix=''): #migrate this to connector
        '''
        create driver chains that basically a direct connection
        '''
        #check attribute list first
        for a in attributeList:
            for j in self.jointList:
                if not j.hasAttr(a):
                    raise Exception('attribute does not exists!')

        driverName = 'drv_'+self.iname + suffix
        #duplicate jnt chain
        drvJnts = duplicateSingleChain(self.jointList)
        #rename
        for dj in drvJnts:
            dj.rename(dj.name().replace('dup_','drv_') + suffix)
        #create driver object
        driverDict = {'driver_jnts' : drvJnts}
        print(drvJnts)
        drv = DriverSystem(driverDict,driverName,None)
        #rearrange master group
        print('start to rearrange')
        masterGrp,masterGroupOffset = drv.getMasterGrpList()
        pm.parent(drv.startJnt,w=True)
        if self.startJnt.getParent() is not None:
            alignTransform(self.startJnt.getParent(),masterGroupOffset)
        else:
            masterGroupOffset.translate.set(0,0,0)
            masterGroupOffset.rotate.set(0,0,0)
            masterGroupOffset.scale.set(1,1,1)
        pm.parent(drv.startJnt,masterGrp)
        #create connector object
        bnManager = self.manager
        drvManager = drv.manager
        drvOutputs = []
        bnInputs = []
        for a in attributeList:
            for dj in drv.driverJnts:
                drvOutput = pm.PyNode(dj.name()+'.'+a)
                drvOutputs.append(drvOutput)
            for j in self.jointList:
                bnInput = pm.PyNode(j.name()+'.'+a)
                bnInputs.append(bnInput)
        con = ConnectorSystem(bnManager,drvManager,drvOutputs,bnInputs,'con_'+self.iname+suffix)
        self.connectorPlugs.append(con.getManager())
        drv.connectorPlugs.append(con.getManager())
        
        return drv,con

    # break connections
    def deleteConnection(self,connectorIndex):
        '''
        this method will delete the connector while retaining the driver
        '''
        connector = self.connectorPlugs[connectorIndex]
        connectorSystem = ConnectorSystem.from_manager(connector)
        connectorSystem._delete()
        self.connectorPlugs.pop(connectorIndex)

    def deleteConnections(self):
        '''
        this method will delete all connectors while retaining the driver
        '''
        for x in range(0,len(self.connectorPlugs)):
            connector = self.connectorPlugs[x]
            connectorSystem = ConnectorSystem.from_manager(connector)
            connectorSystem._delete()
        self.connectorPlugs = [] #clean up

    def deleteDriver(self,connectorIndex):
        '''
        this method will delete the driver and connector system entirely
        '''
        connector = self.connectorPlugs[connectorIndex]
        print(connector)
        connectorSystem = ConnectorSystem.from_manager(connector)
        driverManger = connectorSystem.getDrvManager()
        driverSystem = DriverSystem.from_manager(driverManger)
        connectorSystem._delete()
        driverSystem._delete()

        self.connectorPlugs.pop(connectorIndex)

    def deleteDrivers(self):
        '''
        this method will delete the drivers and connectors system entirely
        '''
        for x in range(0,len(self.connectorPlugs)):
            connector = self.connectorPlugs[x]
            connectorSystem = ConnectorSystem.from_manager(connector)
            driverManger = connectorSystem.getDrvManager()
            driverSystem = DriverSystem.from_manager(driverManger)
            connectorSystem._delete()
            driverSystem._delete()

        self.connectorPlugs = [] #clean up

    # set methods
    def setJointList(self,jointList):
        '''
        set joint list to be handle by this object
        '''
        self.jointList = jointList
        self.jointCount = self.getJointsCount()
        self._setManager(self.manager)

        if len(self.jointList) > 0:
            self.startJnt = self.jointList[0]
            self.endJnt = self.jointList[-1]
            self.parent = self.startJnt.getParent()
        else:
            self.startJnt = None
            self.endJnt = None
            self.parent = None

        self.constructSkinList()


    def setName(self,name):
        '''
        set the internal name for this jointsChain manager node
        '''
        #rename manager
        self.manager.rename('MNG_BOUND_'+name)
        self.iname = self.manager.name().replace('MNG_BOUND_','')
        
        
    
    # get methods
    def name(self):
        '''
        return the internal name for this jointsChain manager node
        '''
        return self.iname

    def getManager(self):
        '''
        return the manager node
        '''
        return self.manager

    def getJointsList(self):
        '''
        return the joints list
        '''
        return self.jointList

    def getJointsCount(self):
        '''
        return the amount of joints in this list
        '''
        return len(self.jointList)

    def getStartJnt(self):
        '''
        return the start joint in this list
        '''
        return self.startJnt

    def getEndJnt(self):
        '''
        return the end joint in this list
        '''
        return self.endJnt

    def getParent(self):
        '''
        return parent of this bound chain
        '''
        return self.parent

    def getDriver(self):
        '''
        return the drivers of this chain
        '''
        return self.driverList
    
    def getConnectorPlugs(self):
        '''
        return the connector object that tied this object to driver type object
        '''
        return self.connectorPlugs

    def getSkinNodeList(self):
        '''
        return the skinNode list that are bound by this object joints chain
        '''
        return self.skinNodeList
        

    def getJointsInfo(self):
        '''
        return the info dict of this joint chain
        '''
        name = self.iname
        manager = self.manager.name()
        jointList = [j.name() for j in self.jointList]
        if self.startJnt is not None:
            startJnt = self.startJnt.name()
        else:
            startJnt = 'NONE'

        if self.endJnt is not None:
            endJnt = self.endJnt.name()
        else:
            endJnt = 'NONE'

        if self.parent is not None:
            parent = self.parent.name()
        else:
            parent = 'NONE'
        jointsCount = self.jointCount
        connectorPlugs = [c.name() for c in self.connectorPlugs]
        skinNodeList = [s.name() for s in self.skinNodeList]

        infoDict = {
            'name' : name,
            'manager' : manager,
            'jointList' : jointList,
            'startJnt' : startJnt,
            'endJnt' : endJnt,
            'parent' : parent,
            'jntCount' : jointsCount,
            'connectorPlugs' : connectorPlugs,
            'skinNodeList' : skinNodeList
        }

        return infoDict


class DriverSystem(object):
    '''
    This base class handles generic driver system
    '''
    def __init__(self,driverDict={'driver_jnts' : []},name='Default',manager=None):
        '''
        Connector should NOT be built from manager
        '''
        self.iname = name
        
        if driverDict.has_key('driver_jnts'):
            self.driverJnts = driverDict['driver_jnts']
            #print(self.driverJnts)
            if len(self.driverJnts) > 0:
                self.startJnt = self.driverJnts[0]
                self.endJnt = self.driverJnts[-1]       
            else:
                raise Exception('No driver joints found!')
            
            self.parent = None
            self.jointCount = self.getJointsCount()
            self.connectorPlugs = []
            if manager is None:
                self.manager = self._createManager(self.iname) #create fresh manager
                self.iname = self.manager.name().replace('MNG_DRIVER_','')
            else:
                self.manager = manager
                self.iname = self.manager.name().replace('MNG_DRIVER_','')
                self.masterGrp,self.masterGrpOffset = self._getMasterGrp()


        else:
            raise Exception('Wrong dictionary for constructing this object')

    #construct method
    @classmethod
    def from_values(cls,*args):
        return cls(args)

    @classmethod
    def from_manager(cls,manager):
        jntList = getJntsFromDriverManager(manager)
        try:
            jntList = reorderSingleChainJointList(jntList)
        except AttributeError:
            pass
        driverDict = {'driver_jnts' : jntList}
        name = manager.name().replace('MNG_DRIVER_','')
        return cls(driverDict,name,manager)

    def _createManager(self,name):
        manager = addDriverManagerNode(name)
        self.manager = manager

        print('start to connect')
        for j in self.driverJnts:
            if not j.hasAttr('driver'):
                j.addAttr('driver')
            print('connect {} to {}'.format(manager,j))
            manager.drvMng >> j.driver
        
        self.masterGrp,self.masterGrpOffset = self.makeMasterGrp() #make new offset group if we create new manager         
        return manager

    def _setManager(self,manager):
        #clear manager first
        clearDriverManager(manager)
        for j in self.driverJnts:
            if not j.hasAttr('driver'):
                j.addAttr('driver')
            manager.drvMng >> j.driver
        self.manager = manager
        self.iname = self.manager.name().replace('MNG_DRIVER_','')
        self.masterGrp,self.masterGrpOffset = self._getMasterGrp()

    def _getMasterGrp(self):
        masterGrp = None
        masterGrpOffset = None
        for n in self.manager.drvMng.outputs():
            if n.hasAttr('driverGrp'):
                self.masterGrp = n
            if n.hasAttr('driverGrpOffset'):
                self.masterGrpOffset = n

        if (masterGrp is None) or (masterGrpOffset is None):
            n_masterGrp,n_masterGrpOffset = self.makeMasterGrp()
            if masterGrp is not None:
                pm.delete(masterGrp)
            if masterGrpOffset is not None:
                pm.delete(masterGrpOffset)
            return n_masterGrp,n_masterGrpOffset
        else:
            return masterGrp,masterGrpOffset


    def _delete(self):
        '''
        delete and clean up this driver system
        '''
        pm.delete(self.manager)
        pm.delete(self.masterGrpOffset)
        self.iname = None
        self.driverJnts = None
        self.startJnt = None
        self.endJnt = None
        self.masterGrp = None
        self.masterGrpOffset = None
        self.parent = None
        self.jointCount = None
        self.connectorPlugs = None
        self.manager = None

    def makeMasterGrp(self):
        '''
        create a master group for this entire system
        '''
        masterGrp = pm.group(empty=True,n='masterGrp_'+self.iname)
        masterGrpOffset = pm.group(n=masterGrp.name()+'Offset')
        #
        alignTransform(self.startJnt,masterGrpOffset)
        pm.parent(self.startJnt,masterGrp)
        #
        masterGrp.addAttr('driverGrp')
        masterGrpOffset.addAttr('driverGrpOffset')
        self.manager.drvMng >> masterGrp.driverGrp
        self.manager.drvMng >> masterGrpOffset.driverGrpOffset
        return masterGrp,masterGrpOffset
        
    # rigging methods
    def createJointController(self,shape='circle'):
        for j in self.driverJnts:
            c = shapeGenerate(shape,'temp_'+j.name())
            cShape = c.getChildren()[0]
            cShape.rename(j.name()+'Shape')
            pm.parent(cShape,j,r=True,s=True)
            pm.delete(c)
    
    # set methods
    def setDriverJnts(self,drvJnts):
        self.driverJnts = drvJnts
        self.jointCount = self.getJointsCount()
        #self._setManager(self.manager)

        if len(self.jointList) > 0:
            self.startJnt = self.jointList[0]
            self.endJnt = self.jointList[-1]
        else:
            self.startJnt = None
            self.endJnt = None

    def setParent(self,parentNode):
        self.parent = parentNode
        pm.parent(self.masterGrpOffset,self.parent)

    # get methods
    def name(self):
        '''
        return the internal name for this jointsChain manager node
        '''
        return self.iname

    def getManager(self):
        '''
        return the manager node
        '''
        return self.manager

    def getJointsList(self):
        '''
        return the joints list
        '''
        return self.driverJnts

    def getJointsCount(self):
        '''
        return the amount of joints in this list
        '''
        return len(self.driverJnts)

    def getStartJnt(self):
        '''
        return the start joint in this list
        '''
        return self.startJnt

    def getEndJnt(self):
        '''
        return the end joint in this list
        '''
        return self.endJnt

    def getParent(self):
        '''
        return parent of this bound chain
        '''
        return self.parent

    def getConnectorPlugs(self):
        '''
        return the drivers of this chain
        '''
        return self.connectorPlugs

    def getMasterGrpList(self):
        '''
        return 2 items, master group and master group offset
        '''
        return self.masterGrp,self.masterGrpOffset

    def getJointsInfo(self):
        '''
        return the info dict of this joint chain
        '''
        name = self.iname
        manager = self.manager.name()
        jointList = [j.name() for j in self.driverJnts]
        if self.startJnt is not None:
            startJnt = self.startJnt.name()
        else:
            startJnt = 'NONE'

        if self.endJnt is not None:
            endJnt = self.endJnt.name()
        else:
            endJnt = 'NONE'

        if self.parent is not None:
            parent = self.parent.name()
        else:
            parent = 'NONE'
        jointsCount = self.jointCount
        masterGrpList = [self.masterGrp.name(),self.masterGrpOffset.name()]
        connectorPlugs = [c.name() for c in self.connectorPlugs]

        infoDict = {
            'name' : name,
            'manager' : manager,
            'jointList' : jointList,
            'startJnt' : startJnt,
            'endJnt' : endJnt,
            'parent' : parent,
            'jntCount' : jointsCount,
            'masterGrpList' : masterGrpList,
            'connectorPlugs' : connectorPlugs,
        }

        return infoDict


class ConnectorSystem(object):
    '''
    This base class handles generic connector system (direct connection)
    '''
    def __init__(self,bnManager,drvManager,drvOutputs=[],bnJntInputs=[],name='Default',manager=None):
        '''
        Connector should NOT be built from manager
        '''
        self.iname = name
        self.bnJntInputs = bnJntInputs
        self.drvOutputs = drvOutputs
        self.bnManager = bnManager
        self.drvManager = drvManager
        #
        self.manager = None
        self.connectorNodes = []

        if manager is None:
            if len(bnJntInputs) == len(drvOutputs):
                self._setup()
            else:
                raise Exception('driver and driven plugs does not match!')
        else:
            self.manager = manager
            self.connectorNodes.append(self.manager.cntMng.outputs()[0])

        #continue tmr, build connector from connector name

    @classmethod
    def from_values(cls,*args):
        return cls(args)

    @classmethod
    def from_manager(cls,manager):
        drvOutputs,bnJntInputs = getIOFromDefaultConnector(manager)[0]
        drvManager = manager.Manager.inputs()[0]
        bnManager = manager.Manager.outputs()[0]
        name = manager.name().replace('MNG_CONNECTOR_','')
        return cls(bnManager,drvManager,drvOutputs,bnJntInputs,name,manager)

    def _setup(self):
        '''
        set up for the generic direct connection
        '''
        connector = pm.group(empty=True,name=('connector_'+self.iname ))
        connector.hiddenInOutliner.set(True)
        for x in range(0,len(self.bnJntInputs)):
            attrName = 'c_'+str(x).zfill(3)
            connector.addAttr(attrName)
            attr = pm.PyNode(connector.name()+'.'+attrName)
            self.drvOutputs[x] >> attr
            attr >> self.bnJntInputs[x]

        self.manager = addConnectorManagerNode(self.iname)
        connector.addAttr('connector')
        self.manager.cntMng >> connector.connector

        self.drvManager.Manager >> self.manager.Manager
        self.manager.Manager >> getEmptyDriverManagerSlot(self.bnManager)

        self.connectorNodes.append(connector)

    def _delete(self):
        '''
        delete and clean up this connector system
        '''
        pm.delete(self.manager)
        pm.delete(self.connectorNodes)

        self.iname = None
        self.bnJntInputs = None
        self.drvOutputs = None
        self.bnManager = None
        self.drvManager = None

    # get methods
    def name(self):
        '''
        return the internal name for this jointsChain manager node
        '''
        return self.iname

    def getBnInputs(self):
        '''
        return the inputs list
        '''
        return self.bnJntInputs

    def getDrvOutputs(self):
        '''
        return the outputs list
        '''
        return self.drvOutputs

    def getBnManager(self):
        '''
        return the bn Manager
        '''
        return self.bnManager

    def getDrvManager(self):
        '''
        return the drv Manager
        '''
        return self.drvManager

    def getManager(self):
        '''
        return the own Manager
        '''
        return self.manager

    def getConnectorsInfo(self):
        '''
        return the info dict of this joint chain
        '''
        name = self.iname
        #manager = self.manager
        bnJntInputs = [i.name() for i in self.bnJntInputs]
        drvJntInputs = [i.name() for i in self.drvOutputs]
        if self.bnManager is not None:
            bnManager = self.bnManager.name()
        else:
            bnManager = 'NONE'

        if self.drvManager is not None:    
            drvManager = self.drvManager.name()
        else:
            drvManager = 'NONE'


        infoDict = {
            'name' : name,
            'inputs' : bnJntInputs,
            'outputs' : drvJntInputs,
            'bound_manager' : bnManager,
            'driver_manager' : drvManager,
        }

        return infoDict


# Rig building functions

