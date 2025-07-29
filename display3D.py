import pygame
import numpy as np
from pygame.locals import DOUBLEBUF, OPENGL, QUIT, KEYDOWN, KEYUP, MOUSEBUTTONDOWN, MOUSEBUTTONUP, MOUSEMOTION
from OpenGL.GL import *
from OpenGL.GLU import gluPerspective
import math


def init_opengl():
    glEnable(GL_DEPTH_TEST)         # Enable depth testing
    glEnable(GL_LIGHTING)           # Enable lighting
    glEnable(GL_LIGHT0)             # Enable light #0
    glEnable(GL_COLOR_MATERIAL)    # Enable color tracking

    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    # Set light properties (position, ambient, diffuse)
    light_pos = [5.0, 5.0, 5.0, 1.0]
    light_ambient = [0.2, 0.2, 0.2, 1.0]
    light_diffuse = [0.7, 0.7, 0.7, 1.0]

    glLightfv(GL_LIGHT0, GL_POSITION, light_pos)
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse)

    glShadeModel(GL_SMOOTH)

def draw_robot():
    # Vertices for the main fuselage (rectangular prism, elongated along X)
    body_length = 0.5
    body_height = 0.2
    body_width  = 0.5

    # Main body vertices (8 corners)
    body_vertices = [
        [-body_length/2, -body_height/2, -body_width/2],  # 0 back-bottom-left
        [ body_length/2, -body_height/2, -body_width/2],  # 1 front-bottom-left
        [ body_length/2,  body_height/2, -body_width/2],  # 2 front-top-left
        [-body_length/2,  body_height/2, -body_width/2],  # 3 back-top-left
        [-body_length/2, -body_height/2,  body_width/2],  # 4 back-bottom-right
        [ body_length/2, -body_height/2,  body_width/2],  # 5 front-bottom-right
        [ body_length/2,  body_height/2,  body_width/2],  # 6 front-top-right
        [-body_length/2,  body_height/2,  body_width/2],  # 7 back-top-right
    ]

    # Normals for each face of the body
    body_normals = [
        [ 0,  0, -1],  # left side
        [ 0,  0,  1],  # right side
        [-1,  0,  0],  # back
        [ 1,  0,  0],  # front
        [ 0,  1,  0],  # top
        [ 0, -1,  0],  # bottom
    ]

    # Faces for main body (each face is a quad, indices in body_vertices)
    body_faces = [
        [0, 3, 2, 1],  # left
        [4, 5, 6, 7],  # right
        [0, 1, 5, 4],  # bottom (under)
        [3, 7, 6, 2],  # top (over)
        [0, 4, 7, 3],  # back
        [1, 2, 6, 5],  # front
    ]

    # Nose cone — simple pyramid at front center
    nose_length = 0.5
    nose_base_width = 0.4
    nose_x = body_length/2

    nose_vertices = [
        [nose_x + nose_length,  0.0,  0.0],                  # tip front
        [nose_x, -nose_base_width/2,  nose_base_width/2],  # base corners
        [nose_x, -nose_base_width/2, -nose_base_width/2],
        [nose_x,  nose_base_width/2, -nose_base_width/2],
        [nose_x,  nose_base_width/2,  nose_base_width/2],
    ]

    nose_faces = [
        [0, 1, 2],  # bottom left triangle
        [0, 2, 3],  # bottom right triangle
        [0, 3, 4],  # top right triangle
        [0, 4, 1],  # top left triangle
        [1, 2, 3, 4], # base quad
    ]

    # Draw main body
    glBegin(GL_QUADS)
    for i, face in enumerate(body_faces):
        glNormal3fv(body_normals[i])
        glColor3fv([0.4, 0.6, 0.8])  # bluish
        for idx in face:
            glVertex3fv(body_vertices[idx])
    glEnd()


    # Draw nose cone (4 triangles + base quad)
    glBegin(GL_TRIANGLES)
    glColor3fv([0.8, 0.1, 0.1])  # reddish nose
    # 4 triangles
    for tri in nose_faces[:4]:
        # Compute face normal manually for nice lighting:
        v0 = np.array(nose_vertices[tri[0]])
        v1 = np.array(nose_vertices[tri[1]])
        v2 = np.array(nose_vertices[tri[2]])
        normal = np.cross(v1 - v0, v2 - v0)
        normal /= np.linalg.norm(normal)
        glNormal3fv(normal)
        for idx in tri:
            glVertex3fv(nose_vertices[idx])
    glEnd()

    # Base quad of nose (optional)
    glBegin(GL_QUADS)
    glNormal3fv([0, -1, 0])  # facing down approx
    for idx in nose_faces[4]:
        glVertex3fv(nose_vertices[idx])
    glEnd()


def draw_ground(size=20, step=1):
    glBegin(GL_LINES)
    for x in range(-size, size + 1, step):
        glVertex3fv([x, 0, -size])
        glVertex3fv([x, 0, size])
    for z in range(-size, size + 1, step):
        glVertex3fv([-size, 0, z])
        glVertex3fv([size, 0, z])
    glEnd()

def quaternion_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y), 0],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x), 0],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y), 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

def normalize(v):
    return v / np.linalg.norm(v)

def axis_angle_to_quaternion(axis, angle_rad):
    axis = normalize(axis)
    s = math.sin(angle_rad / 2.0)
    return np.array([math.cos(angle_rad / 2.0), *(axis * s)], dtype=np.float32)

def get_camera_forward():
    forward_quat = quaternion_multiply(camera_rot, [0, 0, 0, -1])
    forward_quat = quaternion_multiply(forward_quat, [camera_rot[0], -camera_rot[1], -camera_rot[2], -camera_rot[3]])
    return normalize(forward_quat[1:])

def get_camera_directions():
    # Camera forward vector (negative Z in camera space)
    forward = quaternion_multiply(camera_rot, [0, 0, 0, -1])
    forward = quaternion_multiply(forward, [camera_rot[0], -camera_rot[1], -camera_rot[2], -camera_rot[3]])
    forward = normalize(forward[1:])
    
    # Camera right vector (positive X)
    right = quaternion_multiply(camera_rot, [0, 1, 0, 0])
    right = quaternion_multiply(right, [camera_rot[0], -camera_rot[1], -camera_rot[2], -camera_rot[3]])
    right = normalize(right[1:])
    
    # Camera up vector (positive Y)
    up = np.cross(right, forward)
    
    return forward, right, up

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=np.float32)


# Camera state
camera_pos = np.array([0.0, 2.0, 10.0], dtype=np.float32)
camera_rot = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # quaternion
mouse_buttons = [False, False, False]
last_mouse = (0, 0)

def get_camera_matrix():
    rot_matrix = quaternion_to_matrix(camera_rot)
    transform = np.identity(4, dtype=np.float32)
    transform[:3, 3] = -camera_pos
    return rot_matrix @ transform

def handle_mouse_motion(rel):
    global camera_rot, camera_pos
    dx, dy = rel
    dx *= 0.01
    dy *= 0.01

    if mouse_buttons[2]:  # Right mouse drag = rotate
        # Trackball-like rotation
        axis = np.array([-dy, -dx, 0.0], dtype=np.float32)
        angle = np.linalg.norm([dx, dy])
        if angle > 0:
            q_rot = axis_angle_to_quaternion(axis, angle)
            camera_rot = quaternion_multiply(q_rot, camera_rot)

    elif mouse_buttons[1]:  # Middle mouse drag = pan
        # Move along local camera axes
        right = quaternion_multiply(camera_rot, [0, 1, 0, 0])
        up = quaternion_multiply(camera_rot, [0, 0, 1, 0])
        right_vec = np.array(right[1:])
        up_vec = np.array(up[1:])
        camera_pos -= right_vec * dx * 5
        camera_pos += up_vec * dy * 5


def handle_input():
    global camera_pos, camera_yaw, camera_pitch
    move_speed = 0.1  # Adjust to your liking

    keys = pygame.key.get_pressed()
    forward, right, up = get_camera_directions()

    if keys[pygame.K_z]:
        camera_pos += forward * move_speed
    if keys[pygame.K_s]:
        camera_pos -= forward * move_speed
    if keys[pygame.K_q]:
        camera_pos -= right * move_speed
    if keys[pygame.K_d]:
        camera_pos += right * move_speed
    if keys[pygame.K_a]:
        camera_pos -= up * move_speed
    if keys[pygame.K_e]:
        camera_pos += up * move_speed


def Scene3D(positions, quats):
    global mouse_buttons, last_mouse, camera_pos
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("3D Cube with Quaternion Camera")

    glViewport(0, 0, display[0], display[1])
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60, display[0] / display[1], 0.1, 100.0)

    glMatrixMode(GL_MODELVIEW)
    glEnable(GL_DEPTH_TEST)
    glClearColor(0.1, 0.1, 0.1, 1.0)

    init_opengl()

    clock = pygame.time.Clock()
    index = 0
    running = True

    while running:
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        clock.tick(30)
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
            elif event.type == MOUSEBUTTONDOWN:
                if event.button <= 3:
                    mouse_buttons[event.button - 1] = True
                elif event.button == 4:  # Scroll up
                    forward = get_camera_forward()
                    camera_pos += forward * 1.0  # Zoom in
                elif event.button == 5:  # Scroll down
                    forward = get_camera_forward()
                    camera_pos -= forward * 1.0  # Zoom out
            elif event.type == MOUSEBUTTONUP:
                if event.button <= 3:
                    mouse_buttons[event.button - 1] = False
            elif event.type == MOUSEMOTION:
                if any(mouse_buttons):
                    handle_mouse_motion(event.rel)
        handle_input()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glMultMatrixf(get_camera_matrix().T)
        draw_ground()

        if index < len(positions):
            position, quat = positions[index], quats[index]
            glPushMatrix()
            glTranslatef(*position)
            glMultMatrixf(quaternion_to_matrix(quat).T)
            draw_robot()
            glPopMatrix()
            index += 1
        else:
            index = 0
            draw_robot()

        pygame.display.flip()

    pygame.quit()
