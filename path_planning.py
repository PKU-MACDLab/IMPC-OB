import sys
import numpy as np 
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    raise Exception("ompl need to be installed!")
import SET
from geometry import *

## @cond IGNORE
# Our "collision checker". For this demo, our robot's state space
# lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
# centered at (0.5,0.5). Any states lying in this circular region are
# considered "in collision".

class ValidityChecker(ob.StateValidityChecker):
    # Returns whether the given state's position overlaps the self-defined obstacle

    def isValid(self, state):

        if state[0]<=SET.map_range['x'][0] or state[1]<=SET.map_range['y'][0] \
            or state[0]>=SET.map_range['x'][1] or state[1]>=SET.map_range['y'][1]:
            return False
        
        x=int( (state[0]-SET.map_range['x'][0])/SET.resolution )  
        y=int( (state[1]-SET.map_range['y'][0])/SET.resolution )
    
        if SET.path_obstacle_list[x][y]==0:
            return True 
        else:
            return False

## Returns a structure representing the optimization objective to use
#  for optimal motion planning. This method returns an objective
#  which attempts to minimize the length in configuration space of
#  computed paths.
def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)

## Returns an optimization objective which attempts to minimize path
#  length that is satisfied when a path of length shorter than 1.51
#  is found.
def getThresholdPathLengthObj(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(1.51))
    return obj

## Defines an optimization objective which attempts to steer the
#  robot away from obstacles. To formulate this objective as a
#  minimization of path cost, we can define the cost of a path as a
#  summation of the costs of each of the states along the path, where
#  each state cost is a function of that state's clearance from
#  obstacles.
#
#  The class StateCostIntegralObjective represents objectives as
#  summations of state costs, just like we require. All we need to do
#  then is inherit from that base class and define our specific state
#  cost function by overriding the stateCost() method.
#
class ClearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    # Our requirement is to maximize path clearance from obstacles,
    # but we want to represent the objective as a path cost
    # minimization. Therefore, we set each state's cost to be the
    # reciprocal of its clearance, so that as state clearance
    # increases, the state cost decreases.
    def stateCost(self, s):
        return ob.Cost(1 / (self.si_.getStateValidityChecker().clearance(s) +
                            sys.float_info.min))

## Return an optimization objective which attempts to steer the robot
#  away from obstacles.
def getClearanceObjective(si):
    return ClearanceObjective(si)

## Create an optimization objective which attempts to optimize both
#  path length and clearance. We do this by defining our individual
#  objectives, then adding them to a MultiOptimizationObjective
#  object. This results in an optimization objective where path cost
#  is equivalent to adding up each of the individual objectives' path
#  costs.
#
#  When adding objectives, we can also optionally specify each
#  objective's weighting factor to signify how important it is in
#  optimal planning. If no weight is specified, the weight defaults to
#  1.0.
def getBalancedObjective1(si):
    lengthObj = ob.PathLengthOptimizationObjective(si)
    clearObj = ClearanceObjective(si)

    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(lengthObj, 5.0)
    opt.addObjective(clearObj, 1.0)

    return opt

## Create an optimization objective equivalent to the one returned by
#  getBalancedObjective1(), but use an alternate syntax.
#  THIS DOESN'T WORK YET. THE OPERATORS SOMEHOW AREN'T EXPORTED BY Py++.
# def getBalancedObjective2(si):
#     lengthObj = ob.PathLengthOptimizationObjective(si)
#     clearObj = ClearanceObjective(si)
#
#     return 5.0*lengthObj + clearObj


## Create an optimization objective for minimizing path length, and
#  specify a cost-to-go heuristic suitable for this optimal planning
#  problem.
def getPathLengthObjWithCostToGo(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
    return obj


# Keep these in alphabetical order and all lower case
def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


# Keep these in alphabetical order and all lower case
def allocateObjective(si, objectiveType):
    if objectiveType.lower() == "pathclearance":
        return getClearanceObjective(si)
    elif objectiveType.lower() == "pathlength":
        return getPathLengthObjective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return getThresholdPathLengthObj(si)
    elif objectiveType.lower() == "weightedlengthandclearancecombo":
        return getBalancedObjective1(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")



def path_plan(ini,target):

    # Set the log level
    ou.setLogLevel(ou.LOG_WARN)

    runtime=0.03

    objectiveType='PathLength'

    # Construct the robot state space in which we're planning. We're
    # planning in [0,1]x[0,1], a subset of R^2.
    space = ob.RealVectorStateSpace(2)

    lowerbound=min(SET.map_range['x'][0],SET.map_range['y'][0])
    upperbound=max(SET.map_range['x'][1],SET.map_range['y'][1])
    # Set the bounds of space to be in [0,1].
    space.setBounds(lowerbound, upperbound)          
    # bounds = ob.RealVectorBounds
    # bounds.setLow(0， 0.0)
    # bounds.setLow(1, 0.0)
    # bounds.setHigh(0, 100.0)
    # bounds.setHigh(1, 50.0)

    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)

    # Set the object used to check which states in the space are valid
    validityChecker = ValidityChecker(si)
    si.setStateValidityChecker(validityChecker)

    si.setup()

    # Set our robot's starting state
    start = ob.State(space)
    start[0] = ini[0]   # 必须将数组内的每个元素挨个赋值，不能整体赋值（start = leader.ini_p）
    start[1] = ini[1]

    # Set our robot's goal state 
    goal = ob.State(space)
    goal[0] = target[0]
    goal[1] = target[1]

    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)

    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    pdef.setOptimizationObjective(allocateObjective(si, objectiveType))

    # Construct the optimal planner specified by our command line argument.
    # This helper function is simply a switch statement.
    # optimizingPlanner = allocatePlanner(si, plannerType)    
    optimizingPlanner = og.ABITstar(si)

    # Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()         #ompl-base-Planner.cpp 第92行；将setup_置为true

    for i in range(5):
        # attempt to solve the planning problem in the given runtime
        solved = optimizingPlanner.solve(runtime)  #ompl-geometric-SimpleSetup.cpp 第116行;返回值为lastStatus_

        if solved:
            #输出path
            pathpoint_count = pdef.getSolutionPath().getStateCount()        #获取当前路径的状态点个数

            a = pdef.getSolutionPath().printAsMatrix()            #通过printAsMatrix()输出的是str格式的路径点的状态

            b = a.split('\n')                 #通过split将字符串a进行两次划分并转为列表（list），分隔标志为换行符与空格

            c = []
            for i in range(pathpoint_count):
                c.append( b[i].split(' ') )
                c[i] = [float(c[i][0]), float(c[i][1])]   

            return np.array(c)
        else:
            print('cannot find a path, increase runtime try again!')
            runtime=runtime*2
    
    return None
    