from pinocchio import SE3
import numpy as np
from numpy import cross
from numpy.linalg import norm
from pinocchio import SE3, Quaternion,Motion
from pinocchio.utils import rpyToMatrix,rotate
import mlp.config as cfg
from mlp.utils.trajectories import cubicSplineTrajectory,quinticSplineTrajectory
import math
import types

def quatFromConfig(q):
    return Quaternion(q[6],q[3],q[4],q[5])
    

def distPointLine(p_l,x1_l,x2_l):
    p= np.matrix(p_l)
    x1= np.matrix(x1_l)
    x2= np.matrix(x2_l)
    return norm(cross(p-x1,p-x2)) / norm(x2-x1)
    
    

def findPhase(cs,t):
    phase0 = cs.contact_phases[0]
    phasel = cs.contact_phases[-1]
    if t <= phase0.time_trajectory[0]:
        return 0
    elif t >= phasel.time_trajectory[-1]:
        return len(cs.contact_phases)-1


    id = [k for k, phase in enumerate(cs.contact_phases) if t >= phase.time_trajectory[0] and t <= phase.time_trajectory[-1]]
    assert len(id)>=1 or len(id) <= 2
    if len(id) == 2:
        return id[1]
    else:
        return id[0]
    

def SE3toVec(M):
    v = np.matrix(np.zeros((12, 1)))
    for j in range(3):
        v[j] = M.translation[j]
        v[j + 3] = M.rotation[j, 0]
        v[j + 6] = M.rotation[j, 1]
        v[j + 9] = M.rotation[j, 2]
    return v

def MotiontoVec(M):
    v = np.matrix(np.zeros((6, 1)))
    for j in range(3):
        v[j] = M.linear[j]
        v[j + 3] = M.angular[j]
    return v


def SE3FromVec(vect):
    if vect.shape[0] != 12 or vect.shape[1] != 1 :
        raise ValueError("SE3FromVect take as input a vector of size 12")
    placement = SE3.Identity()
    placement.translation=vect[0:3]
    rot = placement.rotation
    if len(rot[:,0].shape) == 1 : # depend if eigenpy.switchToNumpyArray() have been called, FIXME : there should be a better way to check this
        rot[:,0]=np.asarray(vect[3:6]).reshape(-1)
        rot[:,1]=np.asarray(vect[6:9]).reshape(-1)
        rot[:,2]=np.asarray(vect[9:12]).reshape(-1)
    else :
        rot[:,0]=vect[3:6]
        rot[:,1]=vect[6:9]
        rot[:,2]=vect[9:12]
    placement.rotation =rot
    return placement


def MotionFromVec(vect):
    if vect.shape[0] != 6 or vect.shape[1] != 1:
        raise ValueError("MotionFromVec take as input a vector of size 6")
    m = Motion.Zero()
    m.linear= np.matrix(vect[0:3])
    m.angular= np.matrix(vect[3:6])
    return m

def stdVecToMatrix(std_vector):
    if len(std_vector) == 0:
        raise Exception("std_vector is Empty")
    vec_l = []
    for vec in std_vector:
        vec_l.append(vec)

    res = np.hstack(tuple(vec_l))
    return res

## helper method to deal with StateVectorState and other exposed c++ vectors : 
# replace vec[i] with value if it already exist or append the value
def appendOrReplace(vec,i,value):
    assert len(vec) >= i , "There is an uninitialized gap in the vector."
    if i < len(vec) :
        vec[i] = value
    else :
        vec.append(value)

def numpy2DToList(m):
    l = []
    for i in range(m.shape[1]):
        p = m[:, i]
        if len(p.shape) == 1 : # array
            l += [p.tolist()] # TODO : check this
        else : # matrix
            l += [p.T.tolist()[0]]
    return l    

# assume that q.size >= 7 with root pos and quaternion(x,y,z,w)
def SE3FromConfig(q):
    # ~ if isinstance(q,types.ListType):
    if (type(q) is list) or (type(q) is tuple):
        q = np.matrix(q).T
    placement = SE3.Identity()
    placement.translation = q[0:3]
    r = Quaternion(q[6,0],q[3,0],q[4,0],q[5,0])
    placement.rotation = r.matrix()
    return placement

# rotate the given placement of 'angle' (in radian) along axis 'axis'
# axis : either 'x' , 'y' or 'z'
def rotatePlacement(placement,axis,angle):
    T = rotate(axis,angle)
    placement.rotation = placement.rotation*T
    return placement

def rotateFromRPY(placement,rpy):
    trans = SE3.Identity()
    trans.rotation = rpyToMatrix(rpy)
    return placement.act(trans)



def contactPatchForEffector(phase,eeName,Robot = None):
    if not Robot : 
        Robot = cfg.Robot
    if eeName == Robot.rfoot :
        patch = phase.RF_patch
    elif eeName == Robot.lfoot :
        patch = phase.LF_patch
    elif eeName == Robot.rhand :
        patch = phase.RH_patch
    elif eeName == Robot.lhand :
        patch = phase.LH_patch
    else :
        raise Exception("Unknown effector name")
    return patch    

# get the joint position for the given phase with the given effector name
# Note that if the effector is not in contact the phase placement may be uninitialized (==Identity)
def JointPatchForEffector(phase,eeName,Robot=None):
    if not Robot:
        Robot = cfg.Robot
    if eeName == Robot.rfoot :
        patch = phase.RF_patch.copy()
        patch.placement = patch.placement.act(Robot.MRsole_offset.inverse())
    elif eeName == Robot.lfoot :
        patch = phase.LF_patch.copy()
        patch.placement = patch.placement.act(Robot.MLsole_offset.inverse())
    elif eeName == Robot.rhand :
        patch = phase.RH_patch.copy()
        patch.placement = patch.placement.act(Robot.MRhand_offset.inverse())
    elif eeName == Robot.lhand :
        patch = phase.LH_patch.copy()
        patch.placement = patch.placement.act(Robot.MLhand_offset.inverse())
    else :
        raise Exception("Unknown effector name")
    return patch

def JointPlacementForEffector(phase,eeName,Robot=None):
    return JointPatchForEffector(phase,eeName,Robot).placement

def getContactPlacement(phase,eeName,Robot = None):
    if not Robot : 
        Robot = cfg.Robot    
    if eeName == Robot.rfoot :
        return phase.RF_patch.placement
    elif eeName == Robot.lfoot :
        return phase.LF_patch.placement
    elif eeName == Robot.rhand :
        return phase.RH_patch.placement
    elif eeName == Robot.lhand :
        return phase.LH_patch.placement
    else :
        raise Exception("Unknown effector name : "+str(eeName))
    return patch


def isContactActive(phase,eeName,Robot=None):
    if not Robot : 
        Robot = cfg.Robot        
    if eeName == Robot.rfoot :
        return phase.RF_patch.active
    elif eeName == Robot.lfoot :
        return phase.LF_patch.active
    elif eeName == Robot.rhand :
        return phase.RH_patch.active
    elif eeName == Robot.lhand :
        return phase.LH_patch.active
    else :
        raise Exception("Unknown effector name") 

def isContactEverActive(cs,eeName):
    for phase in cs.contact_phases:
        if eeName == cfg.Robot.rfoot :
            if phase.RF_patch.active:
                return True
        elif eeName == cfg.Robot.lfoot :
            if phase.LF_patch.active:
                return True
        elif eeName == cfg.Robot.rhand :
            if phase.RH_patch.active:
                return True
        elif eeName == cfg.Robot.lhand :
            if phase.LH_patch.active:
                return True
        else :
            raise Exception("Unknown effector name") 
    return False
    
def effectorPositionFromHPPPath(fb,problem,eeName,pid,t):
    q = problem.configAtParam(pid,t)
    # compute effector pos from q : 
    fb.setCurrentConfig(q)
    p = fb.getJointPosition(eeName)[0:3] 
    return np.matrix(p).T

def genAMTrajFromPhaseStates(t_init,t_end,init_state,final_state,init_control = None,final_control = None):
    # build Angular moment cubic spline : 
    am_init = init_state[6:9]
    am_end = final_state[6:9]
    if init_control is None:
        dAm_init = np.zeros([3,1])
    else:
        dAm_init = init_control[3:6]
    if final_control is None:
        dAm_end = np.zeros([3,1])      
    else:
        dAm_end = final_control[3:6]
    return cubicSplineTrajectory(t_init,t_end,am_init,dAm_init,am_end,dAm_end)
    
def genCOMTrajFromPhaseStates(t_init,t_end,init_state,final_state,init_control = None,final_control = None):
    # build quintic spline for the com position : 
    p_init = init_state[0:3]
    p_end = final_state[0:3]
    v_init = init_state[3:6]
    v_end = final_state[3:6]
    if init_control is None:
        a_init = np.zeros([3,1])
    else:
        a_init = init_control[0:3]
    if final_control is None:
        a_end = np.zeros([3,1])       
    else:
        a_end = final_control[0:3]
    return quinticSplineTrajectory(t_init,t_end,p_init,v_init,a_init,p_end,v_end,a_end)
    
def connectPhaseTrajToFinalState(phase,duration):
    if duration <= 0.:
        return
    init_state = phase.state_trajectory[-1]
    final_state = phase.final_state
    init_control = phase.control_trajectory[-1]
    t_init = phase.time_trajectory[-1]
    t_end = t_init + duration
    """
    print "# call connectPhaseTrajToFinalState : "
    print "init_state  : ",init_state
    print "final_state : ",final_state
    print "init_control: ",init_control
    print "t_init : ",t_init
    print "t_end  : ",t_end
    """
    com_traj = genCOMTrajFromPhaseStates(t_init,t_end,init_state,final_state,init_control)
    am_traj = genAMTrajFromPhaseStates(t_init,t_end,init_state,final_state,init_control)
    i = len(phase.time_trajectory)
    
    dt = cfg.SOLVER_DT
    t = t_init + dt
    while t < t_end + dt/2. :
        if t > t_end: # may happen due to numerical imprecision
            t = t_end
        c,dc,ddc = com_traj(t)
        L,dL = am_traj(t)
        state = np.matrix(np.zeros(9)).T
        control= np.matrix(np.zeros(6)).T
        state[0:3] = c
        state[3:6] = dc
        control[0:3] = ddc
        state[6:9] = L
        control[3:6] = dL
        phase.state_trajectory.append(state)
        phase.control_trajectory.append(control)
        phase.time_trajectory.append(t)
        t += dt    
    return phase

# fill state_trajectory and control_trajectory in order to connect init_state and final_state of the phase
# use quintic spline for the com position and cubic spline for the angular momentum
# The state_trajectory and control_trajectory in phase should be empty
# and the time_trajectory should start and end at the correct timings
def genSplinesForPhase(phase,current_t,duration,init_control = None):
    assert(len(phase.state_trajectory)) == 0 , "You should only call this method with a 'clean' phase, without any pre existing trajectory"
    assert(len(phase.time_trajectory)) == 0 , "You should only call this method with a 'clean' phase, without any pre existing trajectory"
    # fill the first dt of the trajectories : 
    phase.state_trajectory.append(phase.init_state)
    if init_control:
        phase.control_trajectory.append(init_control)    
    else :
        phase.control_trajectory.append(np.matrix(np.zeros(6)).T)
    phase.time_trajectory.append(current_t)
    return  connectPhaseTrajToFinalState(phase,duration)



def copyPhaseContacts(phase_in,phase_out):
    phase_out.RF_patch = phase_in.RF_patch
    phase_out.LF_patch = phase_in.LF_patch
    phase_out.RH_patch = phase_in.RH_patch
    phase_out.LH_patch = phase_in.LH_patch

def copyPhaseContactPlacements(phase_in,phase_out):
    phase_out.RF_patch.placement = phase_in.RF_patch.placement
    phase_out.LF_patch.placement = phase_in.LF_patch.placement
    phase_out.RH_patch.placement = phase_in.RH_patch.placement
    phase_out.LH_patch.placement = phase_in.LH_patch.placement





def getActiveContactLimbs(phase,Robot=None):
    if not Robot:
        Robot = cfg.Robot
    contacts = []
    if phase.RF_patch.active:
        contacts += [Robot.rLegId]
    if phase.LF_patch.active:
        contacts += [Robot.lLegId]
    if phase.RH_patch.active:
        contacts += [Robot.rArmId]
    if phase.LH_patch.active:
        contacts += [Robot.lArmId]
    return contacts

def createStateFromPhase(fullBody,phase,q = None):
    if q is None:
        q = hppConfigFromMatrice(fullBody.client.robot,phase.reference_configurations[0])
    contacts = getActiveContactLimbs(phase)
    # FIXME : check if q is consistent with the contacts, and project it if not. 
    return fullBody.createState(q,contacts)

def hppConfigFromMatrice(robot,q_matrix):
    q = q_matrix.T.tolist()[0]
    extraDof = robot.getConfigSize() - q_matrix.shape[0]
    assert extraDof >= 0 , "Changes in the robot model happened."
    if extraDof > 0:
        q += [0]*extraDof
    return q

def phasesHaveSameConfig(p0,p1):
    assert len(p0.reference_configurations) > 0 and "CS object given to croc method should store one reference_configuration in each contact_phase"
    assert len(p1.reference_configurations) > 0 and "CS object given to croc method should store one reference_configuration in each contact_phase"
    return np.array_equal(p0.reference_configurations[0],p1.reference_configurations[0])


# TODO : check if only one effector move and raise error
def computeEffectorTranslationBetweenStates(contact_phase,next_phase):
    d = 0
    if(not contact_phase.RF_patch.active and next_phase.RF_patch.active):
        d = next_phase.RF_patch.placement.translation - contact_phase.RF_patch.placement.translation       
    if(not contact_phase.LF_patch.active and next_phase.LF_patch.active):
        d = next_phase.LF_patch.placement.translation - contact_phase.LF_patch.placement.translation       
    if(not contact_phase.LH_patch.active and next_phase.LH_patch.active):
        d = next_phase.LH_patch.placement.translation - contact_phase.LH_patch.placement.translation       
    if(not contact_phase.RH_patch.active and next_phase.RH_patch.active):
        d = next_phase.RH_patch.placement.translation - contact_phase.RH_patch.placement.translation 
    return norm(d)

# TODO : check if only one effector move and raise error
def computeEffectorRotationBetweenStates(contact_phase,next_phase):
    P = np.matrix(np.identity(3))
    Q = np.matrix(np.identity(3))
    if(not contact_phase.RF_patch.active and next_phase.RF_patch.active):
        P = next_phase.RF_patch.placement.rotation 
        Q = contact_phase.RF_patch.placement.rotation       
    if(not contact_phase.LF_patch.active and next_phase.LF_patch.active):
        P = next_phase.LF_patch.placement.rotation 
        Q = contact_phase.LF_patch.placement.rotation 
    if(not contact_phase.LH_patch.active and next_phase.LH_patch.active):
        P = next_phase.LH_patch.placement.rotation 
        Q = contact_phase.LH_patch.placement.rotation 
    if(not contact_phase.RH_patch.active and next_phase.RH_patch.active):
        P = next_phase.RH_patch.placement.rotation 
        Q = contact_phase.RH_patch.placement.rotation
    
    R = P.dot(Q.T)
    tR = R.trace()
    try:
        res = abs(math.acos((tR-1.)/2.))
    except ValueError:
        print ("WARNING : when computing rotation between two contacts, got error : ",e.message)
        print ("With trace value = ",tR)
        res = 0.
    return res

def fullBodyStatesExists(cs,fb):
    lastId = fb.createState([0]*fb.getConfigSize(),[]) - 1 
    if lastId <= 0 : 
        return 0
    else :
        # TODO check with cs if all the states belong to the contact sequence and adjust lastId if necessary
        return lastId
            
def createFullbodyStatesFromCS(cs,fb):
    lastId = fullBodyStatesExists(cs,fb)
    if lastId > 0 :
        print ("States already exist in fullBody instance. endId = ",lastId)
        return 0,lastId
    phase_prev = cs.contact_phases[0]
    beginId = createStateFromPhase(fb,phase_prev)
    lastId = beginId
    print ("CreateFullbodyStateFromCS ##################")
    print ("beginId = ",beginId)
    for pid,phase in enumerate(cs.contact_phases[1:]) : 
        if not phasesHaveSameConfig(phase_prev,phase):
            lastId = createStateFromPhase(fb,phase)
            print ("add phase "+str(pid)+" at state index : "+str(lastId)      )      
            phase_prev = phase
    return beginId, lastId


# fill the given phase with a state, control and time trajectory such that the COM do not moe during all the phase (it stay at it's init_state position)        
def fillPhaseTrajWithZeros(phase,current_t,duration):
    dt = cfg.SOLVER_DT
    state = np.matrix(np.zeros(9)).T
    state[0:3] = phase.init_state[0:3]
    control= np.matrix(np.zeros(6)).T
    t = current_t
    t_end = current_t + duration
    while t < t_end + dt/2. :
        if t > t_end: # may happen due to numerical imprecision
            t = t_end
        phase.state_trajectory.append(state)
        phase.control_trajectory.append(control)
        phase.time_trajectory.append(t)
        t += dt        
        
def computeContactNormal(placement):
    z_up = np.matrix([0., 0., 1.]).T  
    contactNormal = placement.rotation * z_up     
    return contactNormal
    
def computeContactNormalForPhase(phase,eeName):
    return computeContactNormal(getContactPlacement(phase,eeName))

def effectorStatePositionFromWB(Robot,wb_result,id,eeName):
    placement = SE3FromVec(wb_result.effector_references[eeName][:,id])
    pos = placement.act(Robot.dict_offset[eeName]).translation
    vel = wb_result.d_effector_references[eeName][0:3,id]
    acc = wb_result.dd_effector_references[eeName][0:3,id]
    return np.vstack([pos,vel,acc])

def getPhaseEffTrajectoryByName(phase,eeName,Robot):
    if eeName == Robot.rfoot:
        return phase.RF_trajectory
    if eeName == Robot.lfoot:
        return phase.LF_trajectory
    if eeName == Robot.rhand:
        return phase.RH_trajectory
    if eeName == Robot.lhand:
        return phase.LH_trajectory
    raise ValueError("Unknown effector name : "+eeName)


def addEffectorTrajectoryInCS(cs,wb_result,Robot = None):
    if not Robot:
        Robot = cfg.Robot
    for phase in cs.contact_phases:
        for i_traj in range(len(phase.time_trajectory)):
            id = int(phase.time_trajectory[i_traj] / wb_result.dt)
            if id >= len(wb_result.t_t):
                id = len(wb_result.t_t) -1
            for eeName in wb_result.eeNames:
                getPhaseEffTrajectoryByName(phase,eeName,Robot).append(effectorStatePositionFromWB(Robot,wb_result,id,eeName))
    return cs

def rootOrientationFromFeetPlacement(phase,phase_next):
    #FIXME : extract only the yaw rotation
    qr = Quaternion(phase.RF_patch.placement.rotation)
    qr.x = 0 ; qr.y = 0 ; qr.normalize()
    ql = Quaternion(phase.LF_patch.placement.rotation)
    ql.x = 0 ; ql.y = 0 ; ql.normalize()
    q_rot = qr.slerp(0.5, ql)
    placement_init = SE3.Identity()
    placement_init.rotation = q_rot.matrix()
    if phase_next :
        if not isContactActive(phase,cfg.Robot.rfoot)  and isContactActive(phase_next,cfg.Robot.rfoot):
            qr = Quaternion(phase_next.RF_patch.placement.rotation)
            qr.x = 0; qr.y = 0; qr.normalize()
        if not isContactActive(phase, cfg.Robot.lfoot) and isContactActive(phase_next, cfg.Robot.lfoot):
            ql = Quaternion(phase_next.LF_patch.placement.rotation)
            ql.x = 0;ql.y = 0; ql.normalize()
    q_rot = qr.slerp(0.5,ql)
    placement_end = SE3.Identity()
    placement_end.rotation = q_rot.matrix()
    return placement_init,placement_end
