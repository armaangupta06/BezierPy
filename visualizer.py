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



# VEX field height and width in inches.
VEX_FIELD_SIZE = 144

# Ratio of pixels to inches.
INCH_TO_PIXEL = SCREEN_SIZE[0] / VEX_FIELD_SIZE

#Default tangent magnitude value.
magnitude = 0.8

#Radius of point in visualizer in inches.
POINT_RADIUS = 14.5/2 * (INCH_TO_PIXEL)


clock = pygame.time.Clock()
pygame.display.set_caption("Path Visualizer")
fieldImg = pygame.transform.scale(pygame.image.load("./Images/game_field_cropped.png"), SCREEN_SIZE)

running = True
changingMagnitude = False
addingPoint = False
headingInput = 0
magnitudeInput = 0
pointPos = (0, 0)
inputNegative = False
inputDecimal = False
decimalPlaces = 0

editingPoint = False
selectedPoint = None

poses = []
path = curve.path_with_poses(*poses)

fontSize = 18
font = pygame.font.SysFont("comicsans", fontSize)
pointSurface = None
magnitudeSurface = None

while running:
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN and event.key == pygame.K_m and not addingPoint:
            changingMagnitude = True
            print(changingMagnitude)

        if changingMagnitude and event.type == pygame.KEYDOWN:
            if pygame.K_0 <= event.key <= pygame.K_9:
                numInput = event.key - pygame.K_0
                if inputDecimal:
                    magnitudeInput += numInput / (10.0 ** decimalPlaces)
                    decimalPlaces += 1
                else:
                    magnitudeInput = (abs(magnitudeInput) * 10 + numInput) * (1 - 2 * inputNegative)

            if event.key == pygame.K_BACKSPACE:
                magnitudeInput = abs(magnitudeInput)
                if inputDecimal:
                    magnitudeInput = abs(magnitudeInput)
                    magnitudeInput *= (10.0 ** (decimalPlaces-1))
                    magnitudeInput = int((magnitudeInput - (magnitudeInput % 10)) / 10)
                    magnitudeInput *= (1 - 2 * inputNegative)
                    decimalPlaces -= 1
                    magnitudeInput /= (10.0 ** (decimalPlaces-1))

                    if decimalPlaces == 0:
                        inputDecimal = False
                else:
                    magnitudeInput = int((magnitudeInput - (magnitudeInput % 10)) / 10)
                    magnitudeInput *= (1 - 2 * inputNegative)

            if event.key == pygame.K_MINUS:
                inputNegative = not inputNegative
                magnitudeInput = -magnitudeInput
                inputDecimal = False

            if event.key == pygame.K_PERIOD:
                inputDecimal = True
                decimalPlaces = 1

            if event.key == pygame.K_RETURN:
                changingMagnitude = False
                inputDecimal = False
                inputNegative = False
                magnitude = magnitudeInput
                path = curve.generate_points(curve.path_with_poses(*poses, tangent_magnitude=magnitude))
                draw_lines(path)

                magnitudeInput = 0

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
                path = curve.generate_points(curve.path_with_poses(*poses, tangent_magnitude=magnitude))

                headingInput = 0

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                poses = []
                path = []

        if editingPoint and event.type == pygame.MOUSEBUTTONDOWN:
            newPos = pixels_to_coord(event.pos)
            selectedPoint.x = newPos[0]
            selectedPoint.y = newPos[1]
            path = curve.generate_points(curve.path_with_poses(*poses, tangent_magnitude=magnitude))
            editingPoint = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            selectedPoint = clicked_point(event.pos, poses)
            if (selectedPoint != False):
                pointSurface = font.render(f"Moving Point @ ({round(selectedPoint.x, 2)}, {round(selectedPoint.y, 2)})", True, (255, 255, 255), (0, 0, 0))
                editingPoint = True
                changingMagnitude = False
            else:
                pointPos = pixels_to_coord(event.pos)
                pointSurface = font.render(f"Point ({round(pointPos[0], 2)}, {round(pointPos[1], 2)})", True, (255, 255, 255), (0, 0, 0))
                addingPoint = True
                changingMagnitude = False
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

    if changingMagnitude:
        textSurface = font.render(f"Tangent Magnitude: {magnitudeInput}", True, (255, 255, 255), (0, 0, 0))
        screen.blit(textSurface, (0, 0))

    if editingPoint:
        draw_points([selectedPoint], (255, 255, 0))
        screen.blit(pointSurface, (0, 0))

    pygame.display.update()
    clock.tick(30)

pygame.quit()