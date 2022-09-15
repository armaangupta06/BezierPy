import pygame
import curve
from Pose import Pose

def clicked_point(click_pos, poses):
    for pose in poses:
        pose_pos = coord_to_pixels((pose.x, pose.y))
        if pose_pos[0] - POINT_RADIUS < click_pos[0] and pose_pos[0] + POINT_RADIUS > click_pos[0] and pose_pos[1] - POINT_RADIUS < click_pos[1] and pose_pos[1] + POINT_RADIUS > click_pos[1]:
            return pose
    return False

def coord_to_pixels(coords):
    return (coords[0] * INCH_TO_PIXEL + SCREEN_SIZE[0] / 2, coords[1] * -INCH_TO_PIXEL + SCREEN_SIZE[1] / 2)

def pixels_to_coord(pixels):
    return ((pixels[0] - SCREEN_SIZE[0] / 2) / INCH_TO_PIXEL, (pixels[1] - SCREEN_SIZE[1] / 2) / -INCH_TO_PIXEL)

def draw_lines(points):
    point_positions = [coord_to_pixels((point.x, point.y)) for point in points]
    for i in range(len(point_positions) - 1):
        pygame.draw.line(screen, (255, 0, 0), point_positions[i], point_positions[i+1], 5)

def draw_points(poses, color=(0, 255, 0)):
    for pose in poses:
        pygame.draw.circle(screen, color, coord_to_pixels((pose.x, pose.y)), POINT_RADIUS)

pygame.init()

SCREEN_SIZE = (800, 800)
screen = pygame.display.set_mode(SCREEN_SIZE)

POINT_RADIUS = 10

# VEX field height and width in inches
VEX_FIELD_SIZE = 12
# ratio of pixels to inches
INCH_TO_PIXEL = SCREEN_SIZE[0] / VEX_FIELD_SIZE

clock = pygame.time.Clock()
pygame.display.set_caption("Path Visualizer")

fieldImg = pygame.transform.scale(pygame.image.load("./Images/game_field_cropped.png"), SCREEN_SIZE)

running = True

addingPoint = False
headingInput = 0
pointPos = (0, 0)
inputNegative = False

editingPoint = False
selectedPoint = None

poses = []

path = curve.path_with_poses(*poses)

fontSize = 18
font = pygame.font.SysFont("comicsans", fontSize)
pointSurface = None

while running:
    for event in pygame.event.get():
        if addingPoint and event.type == pygame.KEYDOWN:
            # if key is a number key
            if pygame.K_0 <= event.key <= pygame.K_9:
                numInput = event.key - pygame.K_0
                headingInput = (abs(headingInput) * 10 + numInput) * (1 - 2 * inputNegative) 

            if event.key == pygame.K_BACKSPACE:
                headingInput = abs(headingInput)
                headingInput = int((headingInput - (headingInput % 10)) / 10) 
                headingInput *= (1 - 2 * inputNegative)

            if event.key == pygame.K_MINUS:
                # set negative bool incase input is 0
                inputNegative = not inputNegative
                headingInput = -headingInput

            if addingPoint and event.key == pygame.K_RETURN:
                addingPoint = False
                inputNegative = False

                newPose = Pose(pointPos[0], pointPos[1], headingInput)
                poses.append(newPose)
                path = curve.path_with_poses(*poses)

                headingInput = 0

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                poses = []
                path = []

        if editingPoint and event.type == pygame.MOUSEBUTTONDOWN:
            newPos = pixels_to_coord(event.pos)
            selectedPoint.x = newPos[0]
            selectedPoint.y = newPos[1]
            path = curve.path_with_poses(*poses)
            editingPoint = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            selectedPoint = clicked_point(event.pos, poses)
            if (selectedPoint != False):
                pointSurface = font.render(f"Moving Point @ ({round(selectedPoint.x, 2)}, {round(selectedPoint.y, 2)})", True, (255, 255, 255), (0, 0, 0))
                editingPoint = True
            else:
                pointPos = pixels_to_coord(event.pos)
                pointSurface = font.render(f"Point ({round(pointPos[0], 2)}, {round(pointPos[1], 2)})", True, (255, 255, 255), (0, 0, 0))
                addingPoint = True
        if event.type == pygame.QUIT:
            running = False
    

    screen.blit(fieldImg, (0, 0))

    draw_lines(path)
    draw_points(poses)

    if addingPoint:
        pygame.draw.circle(screen, (0, 0, 255), coord_to_pixels(pointPos), POINT_RADIUS)

        textSurface = font.render(f"Heading: {headingInput}", True, (255, 255, 255), (0, 0, 0))
        screen.blit(pointSurface, (0, 0))
        screen.blit(textSurface, (0, pointSurface.get_rect()[3]))

    if editingPoint:
        draw_points([selectedPoint], (255, 255, 0))
        screen.blit(pointSurface, (0, 0))

    pygame.display.update()
    clock.tick(30)

pygame.quit()