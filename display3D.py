# robot_scene.py

import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import time

last_time = time.time()
frame_count = 0
fps = 0

tf_sequence = []
frame_index = 0
frame_index_float = 0
playback_speed = 1

# Camera control
camera_radius = 10
camera_theta = np.pi / 4
camera_phi = np.pi / 4
pan_x, pan_y, pan_z = 0.0, 0.0, 0.0
mouse_prev = [0, 0]
mouse_button = None

follow_robot = False
running = True
step_request = False

def load_tf_sequence(seq):
    global tf_sequence
    tf_sequence = seq

def draw_axes(length=0.5):
    glDisable(GL_LIGHTING)
    glBegin(GL_LINES)
    glColor3f(1, 0, 0)  # X - Red
    glVertex3f(0, 0, 0)
    glVertex3f(length, 0, 0)
    glColor3f(0, 1, 0)  # Y - Green
    glVertex3f(0, 0, 0)
    glVertex3f(0, length, 0)
    glColor3f(0, 0, 1)  # Z - Blue (down)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, -length)
    glEnd()
    glEnable(GL_LIGHTING)

def draw_ground(size=100, step=1):
    glDisable(GL_LIGHTING)
    glBegin(GL_LINES)
    glColor3f(0.6, 0.6, 0.6)  # Make it less reflective
    for i in range(-size, size + 1, step):
        glVertex3f(i, -size, 0)
        glVertex3f(i, size, 0)
        glVertex3f(-size, i, 0)
        glVertex3f(size, i, 0)
    glEnd()
    glEnable(GL_LIGHTING)


def draw_robot(tf):
    x, y, z, roll, pitch, yaw = tf

    # RPY to rotation matrix
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    R = Rz @ Ry @ Rx

    matrix = np.identity(4)
    matrix[:3, :3] = R
    matrix[:3, 3] = [x, y, z]
    matrix = matrix.T.flatten()

    glPushMatrix()
    glMultMatrixf(matrix)
    draw_axes(0.5)
    glColor3f(0.0, 0.4, 1.0)  # Better for lighting
    glutSolidCube(0.4)
    glPopMatrix()

def display():
    global camera_theta, camera_phi, pan_x, pan_y, pan_z, playback_speed

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    if follow_robot and tf_sequence is not None:
        tf = tf_sequence[frame_index % len(tf_sequence)]
        pos = tf[:3]
        # Update center of rotation to robot position
        pan_x, pan_y, pan_z = pos[0], pos[1], pos[2]
    # Convert spherical camera coordinates to Cartesian
    cx = camera_radius * np.sin(camera_phi) * np.cos(camera_theta)
    cy = camera_radius * np.sin(camera_phi) * np.sin(camera_theta)
    cz = camera_radius * np.cos(camera_phi)

    gluLookAt(cx + pan_x, cy + pan_y, -cz + pan_z,
        pan_x, pan_y, pan_z,
        0, 0, -1)

    draw_ground()

    if tf_sequence is not None:
        draw_robot(tf_sequence[frame_index % len(tf_sequence)])

    # Draw FPS at top-left
    glDisable(GL_LIGHTING)
    glColor3f(1, 1, 1)
    draw_text(10, 580, f"FPS: {fps:.1f}")
    draw_text(1080, 580, f"Speed: {playback_speed:.2f}x")

    # Draw TF vector at bottom-center
    if tf_sequence is not None:
        tf = tf_sequence[frame_index % len(tf_sequence)]
        tf_str = "Pos: ({:.2f}, {:.2f}, {:.2f}) RPY: ({:.2f}, {:.2f}, {:.2f})".format(*tf)
        # Center bottom: window width ~800, position text centered at ~400-xoffset
        text_width = len(tf_str) * 9  # approx width per char in pixels
        x_pos = 600 - text_width // 2
        draw_text(x_pos, 10, tf_str)
    glEnable(GL_LIGHTING)

    glutSwapBuffers()

def zoom(direction):
    global camera_radius
    camera_radius *= 0.9 if direction > 0 else 1.1
    camera_radius = np.clip(camera_radius, 1.0, 1000.0)

def draw_text(x, y, text, font=GLUT_BITMAP_HELVETICA_18):
    glWindowPos2f(x, y)  # Positions in window coords (pixels)
    for ch in text:
        glutBitmapCharacter(font, ord(ch))

def keyboard(key, x, y):
    global follow_robot, running, step_request
    if key == b' ':
        # Space toggles play/pause
        running = not running
    elif key == b's' or key == b'S':
        # 's' steps one frame if paused
        running = False
        step_request = True
    elif key == b'r' or key == b'R':
        # 'r' resumes playing
        running = True
    elif key == b'f' or key == b'F':
        follow_robot = not follow_robot
    elif key == b'\x1b':  # ESC key
        print("Exiting simulation...")
        glutLeaveMainLoop()

def special_keys(key, x, y):
    global playback_speed

    if key == GLUT_KEY_LEFT:
        playback_speed = max(1/2**4, playback_speed / 2)
    elif key == GLUT_KEY_DOWN:
        playback_speed = 1.0
    elif key == GLUT_KEY_RIGHT:
        playback_speed = min(2**4, playback_speed * 2)

def update(value):
    global frame_index, frame_count, last_time, fps, running, step_request, playback_speed, frame_index_float
    
    if running or step_request:
        step = playback_speed
        frame_index_float += step
        frame_index = int(frame_index_float)
        step_request = False  # reset step request after stepping

        current_time = time.time()
        elapsed = current_time - last_time
        frame_count += 1
        if elapsed >= 1:
            fps = frame_count / elapsed
            frame_count = 0
            last_time = current_time

    glutPostRedisplay()
    glutTimerFunc(40, update, 0)

def mouse(button, state, x, y):
    global mouse_prev, mouse_button
    if state == GLUT_DOWN:
        mouse_button = button
        mouse_prev = [x, y]
    else:
        mouse_button = None

def motion(x, y):
    global mouse_prev, camera_theta, camera_phi, pan_x, pan_y
    dx = x - mouse_prev[0]
    dy = y - mouse_prev[1]
    mouse_prev = [x, y]

    if mouse_button == GLUT_RIGHT_BUTTON:
        camera_theta += dx * 0.005
        camera_phi -= dy * 0.005
        camera_phi = np.clip(camera_phi, 0.01, np.pi - 0.01)
    elif mouse_button == GLUT_MIDDLE_BUTTON:
        # Convert to 3D panning based on camera orientation
        cam_x = camera_radius * np.sin(camera_phi) * np.cos(camera_theta)
        cam_y = camera_radius * np.sin(camera_phi) * np.sin(camera_theta)
        cam_z = camera_radius * np.cos(camera_phi)

        # View direction
        forward = np.array([-cam_x, -cam_y, cam_z])
        forward /= np.linalg.norm(forward)

        # Right = forward x up
        up = np.array([0, 0, -1])
        right = np.cross(forward, up)
        right /= np.linalg.norm(right)

        # True up vector
        true_up = np.cross(right, forward)

        # Scale mouse movement into world space
        factor = 0.01
        move = right * (-dx * factor) + true_up * (dy * factor)

        # Apply to pan
        global pan_x, pan_y, pan_z
        pan_x += move[0]
        pan_y += move[1]
        pan_z += move[2]

def init():
    glClearColor(0.7, 0.7, 0.7, 1.0)
    glEnable(GL_DEPTH_TEST)

    # --- Lighting ---
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    
    # Position light above and in front of scene
    light_position = [10.0, 10.0, 10.0, 1.0]  # w=1.0 means positional
    glLightfv(GL_LIGHT0, GL_POSITION, light_position)

    # Light color (white diffuse light)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
    glLightfv(GL_LIGHT0, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0])

    # Enable color tracking
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    # Optional: Add slight shininess
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 32)

    glMatrixMode(GL_PROJECTION)
    gluPerspective(60, 2.0, 0.1, 1000.0)
    glMatrixMode(GL_MODELVIEW)


def run_viewer(sequence):
    load_tf_sequence(sequence)
    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(1200, 600)
    glutCreateWindow(b"3D Robot Viewer")
    init()
    glutDisplayFunc(display)
    glutTimerFunc(50, update, 0)
    glutMouseFunc(mouse)
    glutMotionFunc(motion)
    # Register your keyboard handler here:
    glutKeyboardFunc(keyboard)
    glutSpecialFunc(special_keys)

    try:
        glutMouseWheelFunc(lambda wheel, direction, x, y: zoom(direction))
    except:
        print("Mouse wheel not supported; zoom with +/- keys instead.")
    glutMainLoop()
