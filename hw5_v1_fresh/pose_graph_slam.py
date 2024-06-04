import gtsam.noiseModel
import numpy as np
import gtsam
from typing import NamedTuple

def pretty_print(arr):
    return '\n'.join([' '.join(['%.2f' % x for x in c]) for c in arr])

class Pose2(NamedTuple):
    '''
    Pose2 class for 2D pose
    @usage: pose = Pose2(id, x, y, z)
            print(pose.x)
    '''
    id: int
    x: float
    y: float
    theta: float

class Edge2(NamedTuple):
    '''
    Edge2 class for 2D edge
    @usage: edge = Edge2(id1, id2, x, y, z, info)
            print(edge.x)
    '''
    id1: int
    id2: int
    x: float
    y: float
    theta: float
    info: np.ndarray # 3x3 matrix

    def __str__(self):
        return f"Edge2(id1={self.id1}, id2={self.id2}, x={self.x}, y={self.y}, theta={self.theta},\ninfo=\n{pretty_print(self.info)})\n"

class Pose3(NamedTuple):
    '''
    Pose3 class for 3D pose
    @usage: pose = Pose3(id, x, y, z, qx, qy, qz, qw)
            print(pose.x)
    '''
    id: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

class Edge3(NamedTuple):
    '''
    Edge3 class for 3D edge
    @usage: edge = Edge3(id1, id2, x, y, z, qx, qy, qz, qw, info)
            print(edge.x)
    '''
    id1: int
    id2: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    info: np.ndarray # 6x6 matrix

    def __str__(self):
        return f"Edge3(id1={self.id1}, id2={self.id2}, x={self.x}, y={self.y}, z={self.z}, qx={self.qx}, qy={self.qy}, qz={self.qz}, qw={self.qw},\ninfo=\n{pretty_print(self.info)})\n"


def read_g2o_2d(file_name):
    data = {
        'poses': [],
        'edges': []
    }

    # read the file
    with open(file_name, "r") as f:
        lines = f.readlines()

        #############################################################################
        #                    TODO: Implement your code here                         #
        #############################################################################

        # fill in the `data` dict with Pose2 or Edge2 objects
        # ...
        for line in lines:
            segments = line.split()
            if segments[0] == 'VERTEX_SE2':
                #print("Hi there! VERTEX_SE2")
                id = int(segments[1])
                x = float(segments[2])
                y = float(segments[3])
                theta = float(segments[4])
                pose = Pose2(id, x, y, theta)
                data['poses'].append(pose)
            elif segments[0] == 'EDGE_SE2':
                '''
                Since we are given that the matrix
                is invertible, we don't need to verify
                that.
                '''

                id1 = int(segments[1])
                id2 = int(segments[2])
                x = float(segments[3])
                y = float(segments[4])
                theta = float(segments[5])
                q_11 = float(segments[6])
                q_12 = float(segments[7])
                q_13 = float(segments[8])
                q_22 = float(segments[9])
                q_23 = float(segments[10])
                q_33 = float(segments[11])
                info = np.array([[q_11, q_12, q_13],
                                 [q_12, q_22, q_23],
                                 [q_13, q_23, q_33]])
                edge = Edge2(id1,id2,x,y,theta,info)
                data['edges'].append(edge)
        #############################################################################
        #                            END OF YOUR CODE                               #
        #############################################################################
    return data

def gn_2d(data):
    poses = data['poses']
    edges = data['edges']
    #print(poses)
    #print(edges)
    result = gtsam.Values()
    #print(result)
    # use this covariance for the prior factor of the first pose
    first_pose_prior_cov = np.array([0.5, 0.5, 0.1])

    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    # create an empty factor graph
    graph = gtsam.NonlinearFactorGraph()
    initial_values = gtsam.Values()

    # set initial_values according to poses
    for pose in poses:
        # Extract pose information
        pose_id = pose.id
        x = pose.x
        y = pose.y
        theta = pose.theta

        # Create pose_b object
        pose_b = gtsam.Pose2(x,y,theta)
        print(pose_b)
        # Add pose to initial values
        initial_values.insert(pose_id, pose_b)
        print(initial_values)

    # add prior factor for the first pose
    first_pose = poses[0]
    #print(first_pose)
    prior_Noise = gtsam.noiseModel.Diagonal.Variances(first_pose_prior_cov)
    prior_Mean = gtsam.Pose2(first_pose.x, first_pose.y, first_pose.theta)
    graph.add(gtsam.PriorFactorPose2(first_pose.id, prior_Mean , prior_Noise))

    # add between factors according to edges
    for edge in edges:
        # extract edge information
        id1 = edge.id1
        id2 = edge.id2
        x_edge = edge.x
        y_edge = edge.y
        theta_edge = edge.theta
        info = edge.info
    
    # Create a noise model from information matrix
        measurement_noise = gtsam.noiseModel.Gaussian.Information(info)
        relative_pose = gtsam.Pose2(x_edge,y_edge,theta_edge)
        graph.add(gtsam.BetweenFactorPose2(id1, id2, relative_pose, measurement_noise))

    # optimize the graph
    optimizer = gtsam.GaussNewtonOptimizer(graph, initial_values)
    result = optimizer.optimize()

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################

    # return the poses
    return gtsam.utilities.extractPose2(result)

def isam_2d(data):
    poses = data['poses']
    edges = data['edges']
    #print(poses)
    #print(edges)
    result = gtsam.Values()
    # use this covariance for the prior factor of the first pose
    first_pose_prior_cov = np.array([0.5, 0.5, 0.1])

    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    # create optimizer
    # ...
    isam = gtsam.ISAM2()
    for pose in poses:

        frame_id = pose.id

        # create an empty factor graph
        graph = gtsam.NonlinearFactorGraph()
        initial_values = gtsam.Values()
        
        if frame_id==0:
            # initialization
            prior_Noise = gtsam.noiseModel.Diagonal.Variances(first_pose_prior_cov)
            prior_Mean = gtsam.Pose2(pose.x, pose.y, pose.theta)
            graph.add(gtsam.PriorFactorPose2(pose.id, prior_Mean , prior_Noise))
            initial_values.insert(frame_id, prior_Mean)
            #pass
        else:
            # optimize new frame
            prev_Pose = result.atPose2(frame_id-1)
            initial_values.insert(frame_id,prev_Pose)
            #pass
            for edge in edges:
                edge_id1 = edge.id1
                edge_id2 = edge.id2
                if edge_id2 == frame_id:
                    covariance = np.linalg.inv(edge.info)
                    model = gtsam.noiseModel.Gaussian.Covariance(covariance) # not sure.
                    graph.add(gtsam.BetweenFactorPose2(edge_id1, edge_id2, gtsam.Pose2(edge.x, edge.y, edge.theta), model))

        # update isam
        # ...
        isam.update(graph, initial_values)
        result = isam.calculateEstimate()
        # result = ...

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################

    # return the poses
    return gtsam.utilities.extractPose2(result)

def read_g2o_3d(file_name):
    data = {
        'poses': [],
        'edges': []
    }


    # read the file
    with open(file_name, "r") as f:
        lines = f.readlines()

        #############################################################################
        #                    TODO: Implement your code here                         #
        #############################################################################

        # fill in the `data` dict with Pose3 or Edge3 objects
        for line in lines:
            segments = line.split()
            test = segments[0]
            if test[0] == 'V':
                id = int(segments[1])
                x = float(segments[2])
                y = float(segments[3])
                z = float(segments[4])
                qx = float(segments[5])
                qy = float(segments[6])
                qz = float(segments[7])
                qw = float(segments[8])
                pose = Pose3(id, x, y, z, qx, qy, qz, qw)
                data['poses'].append(pose)
            elif test[0] == 'E':
                ID = int(segments[1])
                ID2 = int(segments[2])
                x = float(segments[3])
                y = float(segments[4])
                z = float(segments[5])
                qx = float(segments[6])
                qy = float(segments[7])
                qz = float(segments[8])
                qw = float(segments[9])

                #21 more to go.
                a,b,c,d,e,f2 = float(segments[10]), float(segments[11]), float(segments[12]), float(segments[13]), float(segments[14]), float(segments[15])
                g,h,i,j,k,l = float(segments[16]), float(segments[17]), float(segments[18]), float(segments[19]), float(segments[20]), float(segments[21])
                m,n,o,p,q,r = float(segments[22]), float(segments[23]), float(segments[24]), float(segments[25]), float(segments[26]), float(segments[27])
                s, t, u = float(segments[28]), float(segments[29]), float(segments[30])

                omega = np.array([[a, b, c, d, e, f2],
                                 [b, g, h, i, j, k],
                                 [c, h, l, m, n, o],
                                 [d, i, m, p, q, r],
                                 [e, j, n, q, s, t],
                                 [f2, k, o, r, t, u]])
                edge = Edge3(ID, ID2, x, y, z, qx, qy, qz, qw, omega)
                data['edges'].append(edge)  
        # ...

        #############################################################################
        #                            END OF YOUR CODE                               #
        #############################################################################
    
    return data  

def gn_3d(data):
    poses = data['poses']
    edges = data['edges']
    result = gtsam.Values()

    # use this covariance for the prior factor of the first pose
    first_pose_prior_cov = np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1])

    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    # create an empty factor graph
    graph = gtsam.NonlinearFactorGraph()
    initial_values = gtsam.Values()

    # set initial_values according to poses
    for pose in poses:
        pose_id = pose.id
        x = pose.x
        y = pose.y
        z = pose.z
        qx = pose.qx
        qy = pose.qy
        qz = pose.qz
        qw = pose.qw
        translation = gtsam.Point3(x,y,z)
        rotation = gtsam.Rot3(qw, qx, qy, qz)
        pose_b = gtsam.gtsam.Pose3(rotation,translation) # Not sure if qw needs to be there.
        initial_values.insert(pose_id, pose_b)

    # add prior factor for the first pose
    # ...
    first_pose = poses[0]
    prior_Noise = gtsam.noiseModel.Diagonal.Variances(first_pose_prior_cov)
    #prior_Mean = gtsam.Pose3(first_pose.x, first_pose.y, first_pose.z, first_pose.qx, first_pose.qy, first_pose.qz, first_pose.qw)
    prior_translate = gtsam.Point3(first_pose.x, first_pose.y, first_pose.z)
    prior_rotate = gtsam.Rot3(first_pose.qw, first_pose.qx, first_pose.qy, first_pose.qz)
    prior_Mean = gtsam.gtsam.Pose3(prior_rotate, prior_translate)
    graph.add(gtsam.PriorFactorPose3(first_pose.id, prior_Mean, prior_Noise))    


    # add between factors according to edges
    # ...
    for edge in edges:
        id1 = edge.id1
        id2 = edge.id2
        x_edge = edge.x
        y_edge = edge.y
        z_edge = edge.z
        qx_edge = edge.qx
        qy_edge = edge.qy
        qz_edge = edge.qz
        qw_edge = edge.qw
        info = edge.info

        measurement_noise = gtsam.noiseModel.Gaussian.Information(info)
        relative_translation = gtsam.Point3(x_edge, y_edge, z_edge)
        relative_rotation = gtsam.Rot3(qw_edge, qx_edge, qy_edge, qz_edge)
        relative_pose = gtsam.gtsam.Pose3(relative_rotation, relative_translation)
        graph.add(gtsam.BetweenFactorPose3(id1, id2, relative_pose, measurement_noise))

    # optimize the graph
    optimizer = gtsam.GaussNewtonOptimizer(graph, initial_values)
    result = optimizer.optimize()

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################

    # return the poses
    return gtsam.utilities.extractPose3(result)

def isam_3d(data):
    poses = data['poses']
    edges = data['edges']
    result = gtsam.Values()

    # use this covariance for the prior factor of the first pose
    first_pose_prior_cov = np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1])

    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    # create optimizer
    isam = gtsam.ISAM2()

    for pose in poses:

        frame_id = pose.id

        # create an empty factor graph
        graph = gtsam.NonlinearFactorGraph()
        initial_values = gtsam.Values()
        
        if frame_id==0:
            # initialization

            # Notice that the order of quaternion is different from
            # the one in the g2o file. GTSAM uses (qw, qx, qy, qz).

            prior_Noise = gtsam.noiseModel.Diagonal.Variances(first_pose_prior_cov)
            prior_translate = gtsam.Point3(pose.x, pose.y, pose.z)
            prior_rotate = gtsam.Rot3(pose.qw, pose.qx, pose.qy, pose.qz)
            prior_Mean = gtsam.gtsam.Pose3(prior_rotate, prior_translate)
            graph.add(gtsam.PriorFactorPose3(frame_id, prior_Mean, prior_Noise))
            initial_values.insert(frame_id,prior_Mean)

            #pass
        else:
            # optimize new frame
            prev_Pose = result.atPose3(frame_id-1)
            initial_values.insert(frame_id, prev_Pose)
            #pass
            for edge in edges:
                edge_id1 = edge.id1
                edge_id2 = edge.id2
                if edge_id2 == frame_id:
                    covariance = np.linalg.inv(edge.info)
                    model = gtsam.noiseModel.Gaussian.Covariance(covariance)
                    added_translation = gtsam.Point3(edge.x, edge.y, edge.z)
                    added_rotation = gtsam.Rot3(edge.qw, edge.qx, edge.qy, edge.qz)
                    graph.add(gtsam.BetweenFactorPose3(edge_id1, edge_id2, gtsam.gtsam.Pose3(added_rotation, added_translation), model))
        # update isam
        isam.update(graph,initial_values)
        result = isam.calculateEstimate()

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################

    # return the poses
    return gtsam.utilities.extractPose3(result)
