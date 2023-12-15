from ...state import AllState,ObjectPose,ObjectFrameEnum,VehicleState
from ..component import Component

class StandardPerceptionNormalizer(Component):
    """Updates the start pose and converts all objects to the current vehicle
    frame, in preparation for planning."""
    def rate(self):
        return 10.0
    def state_inputs(self):
        return ['all']
    def state_outputs(self):
        return []
    def update(self,state : AllState):
        if state.start_vehicle_pose is None:
            if state.vehicle != VehicleState.zero():
                state.start_vehicle_pose = state.vehicle.pose
            else:
                return
            
        #convert vehicle pose to start frame
        state.vehicle.pose = state.vehicle.pose.to_frame(ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose)

        #convert roadgraph to current frame
        state.roadgraph = state.roadgraph.to_frame(ObjectFrameEnum.CURRENT, current_pose=state.vehicle_pose, start_pose_abs=state.start_vehicle_pose)

        for k,a in state.agents.items():
            a.pose = a.pose.to_frame(ObjectFrameEnum.CURRENT, current_pose=state.vehicle_pose, start_pose_abs=state.start_vehicle_pose)
        
        for k,o in state.obstacles.items():
            o.pose = o.pose.to_frame(ObjectFrameEnum.CURRENT, current_pose=state.vehicle_pose, start_pose_abs=state.start_vehicle_pose)

        #convert predictions to current frame.        
        for k,a in state.agent_intents.items():
            for pred in a.predictions:
                if pred.path is not None:
                    for i,p in enumerate(pred.path):
                        pred.path[i] = p.to_frame(ObjectFrameEnum.CURRENT, current_pose=state.vehicle_pose, start_pose_abs=state.start_vehicle_pose)

        #TODO: advance agent predictions and paths to current vehicle time