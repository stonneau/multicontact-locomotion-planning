try:
    from sl1m.planner import *
except ImportError:
    message = "ERROR: Cannot import SL1M python library.\n"
    message += "Did you correctly installed it?\n"
    message +="See https://gepgitlab.laas.fr/loco-3d/sl1m"
    raise ImportError(message)
from pinocchio.utils import *
import importlib
import multicontact_api
from multicontact_api import ContactSequence
from mlp.utils.cs_tools import addPhaseFromConfig, setFinalState
from mlp.viewer.display_tools import initScene, displaySteppingStones
from pinocchio.utils import matrixToRpy
from pinocchio import Quaternion, SE3
# ~ from hpp.corbaserver.rbprm.tools.surfaces_from_path import getSurfacesFromGuideContinuous
import random
from mlp.utils.requirements import Requirements
multicontact_api.switchToNumpyArray()

class ContactOutputsSl1mQuadruped(Requirements, comConstraintsFun, relativeKinConstraintsFun, nEffectors = 4):
    consistentContacts = True

Z_AXIS = np.array([0, 0, 1])
VERBOSE = False
EPS_Z = 0.005  # offset added to feet z position, otherwise there is collisions with the ground

##### MOVE the next methods to a robot-specific file : #####
import os
####################################


def normal(phase):
    s = phase["S"][0]
    n = cross(s[:, 1] - s[:, 0], s[:, 2] - s[:, 0])
    n /= norm(n)
    if n[2] < 0.:
        for i in range(3):
            n[i] = -n[i]
    # ~ print "normal ", n
    return n


def quatConfigFromMatrix(m):
    quat = Quaternion(m)
    return quatToConfig(quat)


def quatToConfig(quat):
    return [quat.x, quat.y, quat.z, quat.w]


def gen_pb(root_init, R, surfaces, ref_root_height):
    #print "surfaces = ",surfaces
    print("number of surfaces : ", len(surfaces))
    print("number of rotation matrix for root : ", len(R))
    nphases = len(surfaces)        
    p0 = None
    res = { "p0" : p0, "c0" : None, "nphases": nphases}   
    #TODO : relative kin for different normals ... 
    phaseData = [ {"K" : [comConstraintsFun(R[i]) for _ in range(len(surfaces[i]))], "S" : surfaces[i], "allRelativeK" : [relativeKinConstraintsFun(R[i])], "rootOrientation": R[i], "S": surfaces[i] } for i in range(nphases)]
    res["phaseData"] = phaseData
    return res


def solve(planner, guide_step_size, guide_max_yaw, ref_root_height, initCom = None, initPos = None, endCom = None):
    from sl1m.planner_l1_generic_equalities_as_ineq import solveMIPGurobi, plotQPRes, initGlobals 
    initGlobals(nEffectors = nEffectors)  
    success = False
    maxIt = 50
    it = 0
    defaultStep = guide_step_size
    step = defaultStep
    variation = 0.4  # FIXME : put it in config file, +- bounds on the step size
    if hasattr(planner, "pathId"):
        pathId = planner.pathId
    elif hasattr(planner, "pId"):
        pathId = planner.pId
    else:
        pathId = planner.ps.numberPaths() - 1
    while not success and it < maxIt:
        if it > 0:
            step = defaultStep + random.uniform(-variation, variation)
        #configs = getConfigsFromPath (planner.ps, planner.pathId, step)
        #getSurfacesFromPath(planner.rbprmBuilder, configs, surfaces_dict, planner.v, True, False)
        viewer = planner.v
        if not hasattr(viewer, "client"):
            viewer = None
        R, surfaces = getSurfacesFromGuideContinuous(planner.rbprmBuilder,
                                                     planner.ps,
                                                     planner.afftool,
                                                     pathId,
                                                     viewer,
                                                     step,
                                                     useIntersection=True,
                                                     max_yaw=guide_max_yaw)
        pb = gen_pb(planner.q_init, R, surfaces, ref_root_height)
        try:
            # ~ pb, coms, footpos, allfeetpos, res = solveL1(pb, surfaces, None)
            # ~ pb, coms, footpos, allfeetpos, res = solve(initCom = initCom, initPos = initPos)
            pb, res, time = solveMIPGurobi(pb, surfaces, MIP = True, draw_scene = None, plot = True, l1Contact = False, initPos = initPos, endPos = endPos, initCom = initCom, endCom=  endCom)
            coms, footpos, allfeetpos = retrieve_points_from_res(pb, res)
            success = True
        except:
            print("## Planner failed at iter : " + str(it) + " with step length = " + str(step))
        it += 1
    if not success:
        raise RuntimeError("planner always fail.")
    return pathId, pb, coms, footpos, allfeetpos, res


def runLPFromGuideScript(cfg):
    #the following script must produce a
    if hasattr(cfg, 'SCRIPT_ABSOLUTE_PATH'):
        scriptName = cfg.SCRIPT_ABSOLUTE_PATH
    else:
        scriptName = cfg.RBPRM_SCRIPT_PATH + "." + cfg.SCRIPT_PATH + '.' + cfg.DEMO_NAME
    scriptName += "_path"
    print("Run Guide script : ", scriptName)
    module = importlib.import_module(scriptName)
    planner = module.PathPlanner()
    planner.run()
    # compute sequence of surfaces from guide path
    pathId, pb, coms, footpos, allfeetpos, res = solve(planner, cfg.GUIDE_STEP_SIZE, cfg.GUIDE_MAX_YAW,
                                                       cfg.IK_REFERENCE_CONFIG[2])
    root_init = planner.ps.configAtParam(pathId, 0.001)[0:7]
    root_end = planner.ps.configAtParam(pathId, planner.ps.pathLength(pathId) - 0.001)[0:7]
    return RF, root_init, root_end, pb, coms, footpos, allfeetpos, res


def runLPScript(cfg):
    #the following script must produce a
    if hasattr(cfg, 'SCRIPT_ABSOLUTE_PATH'):
        scriptName = cfg.SCRIPT_ABSOLUTE_PATH
    else:
        scriptName = 'scenarios.' + cfg.SCRIPT_PATH + '.' + cfg.DEMO_NAME
    print("Run LP script : ", scriptName)
    cp = importlib.import_module(scriptName)
    pb, coms, footpos, allfeetpos, res = cp.solve()
    root_init = cp.root_init[0:7]
    root_end = cp.root_end[0:7]
    return cp.RF, root_init, root_end, pb, coms, footpos, allfeetpos, res


def generate_contact_sequence_sl1m(cfg):
    #RF,root_init,pb, coms, footpos, allfeetpos, res = runLPScript(cfg)
    RF, root_init, root_end, pb, coms, footpos, allfeetpos, res = runLPFromGuideScript(cfg)
    multicontact_api.switchToNumpyArray()
    # load scene and robot
    fb, v = initScene(cfg.Robot, cfg.ENV_NAME, True)
    q_init = cfg.IK_REFERENCE_CONFIG.tolist() + [0] * 6
    q_init[0:7] = root_init
    feet_height_init = allfeetpos[0][2]
    print("feet height initial = ", feet_height_init)
    q_init[2] = feet_height_init + cfg.IK_REFERENCE_CONFIG[2]
    q_init[2] += EPS_Z
    #q_init[2] = fb.referenceConfig[2] # 0.98 is in the _path script
    if v:
        v(q_init)

    # init contact sequence with first phase : q_ref move at the right root pose and with both feet in contact
    # FIXME : allow to customize that first phase
    cs = ContactSequence(0)
    addPhaseFromConfig(fb, cs, q_init, [fb.rLegId, fb.lLegId])

    # loop over all phases of pb and add them to the cs :
    for pId in range(2, len(pb["phaseData"])):  # start at 2 because the first two ones are already done in the q_init
        prev_contactPhase = cs.contactPhases[-1]
        #n = normal(pb["phaseData"][pId])
        phase = pb["phaseData"][pId]
        moving = phase["moving"]
        movingID = fb.lfoot
        if moving == RF:
            movingID = fb.rfoot
        pos = allfeetpos[pId]  # array, desired position for the feet movingID
        pos[2] += EPS_Z  # FIXME it shouldn't be required !!
        # compute desired foot rotation :
        if cfg.SL1M_USE_ORIENTATION:
            quat0 = Quaternion(pb["phaseData"][pId]["rootOrientation"])
            if pId < len(pb["phaseData"]) - 1:
                quat1 = Quaternion(pb["phaseData"][pId + 1]["rootOrientation"])
            else:
                quat1 = Quaternion(pb["phaseData"][pId]["rootOrientation"])
            if cfg.SL1M_USE_INTERPOLATED_ORIENTATION :
                rot = quat0.slerp(0.5, quat1)
                # check if feets do not cross :
                if moving == RF:
                    qr = rot
                    ql = Quaternion(prev_contactPhase.contactPatch(fb.lfoot).placement.rotation)
                else:
                    ql = rot
                    qr = Quaternion(prev_contactPhase.contactPatch(fb.rfoot).placement.rotation)
                rpy = matrixToRpy((qr * (ql.inverse())).matrix())  # rotation from the left foot pose to the right one
                if rpy[2] > 0:  # yaw positive, feet are crossing
                    rot = quat0  # rotation of the root, from the guide
            else:
                rot = quat0  # rotation of the root, from the guide
        else:
            rot = Quaternion.Identity()
        placement = SE3()
        placement.translation = np.array(pos).T
        placement.rotation = rot.matrix()
        cs.moveEffectorToPlacement(movingID, placement)

    # final phase :
    # fixme : assume root is in the middle of the last 2 feet pos ...
    q_end = cfg.IK_REFERENCE_CONFIG.tolist() + [0] * 6
    #p_end = (allfeetpos[-1] + allfeetpos[-2]) / 2.
    #for i in range(3):
    #    q_end[i] += p_end[i]
    q_end[0:7] = root_end
    feet_height_end = allfeetpos[-1][2]
    print("feet height final = ", feet_height_end)
    q_end[2] = feet_height_end + cfg.IK_REFERENCE_CONFIG[2]
    q_end[2] += EPS_Z
    fb.setCurrentConfig(q_end)
    com = fb.getCenterOfMass()
    setFinalState(cs, com, q=q_end)
    if cfg.DISPLAY_CS_STONES:
        displaySteppingStones(cs, v.client.gui, v.sceneName, fb)

    return cs, fb, v
