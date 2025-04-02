import pygame
import json
import time

pygame.init()
SCALE = 20
FRAME_RATE = 10
SCREEN_SIZE = (800, 600)  # Adjust as needed
screen = pygame.display.set_mode(SCREEN_SIZE)
pygame.display.set_caption("Timestamp Animation")
clock = pygame.time.Clock()

data = None

with open("arrow_shape", "r") as file:
    data = json.load(file)

timestamps = sorted([int(k) for k in data.keys()]) # convert to int for sorting
print(timestamps)
current_timestamp_index = 0

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255, 255, 255))

    if timestamps and current_timestamp_index < len(timestamps):
        current_timestamp = timestamps[current_timestamp_index]
        sim_record = data[str(current_timestamp)]

        for pos in sim_record:
            pygame.draw.circle(screen, (0, 0, 255), (int(pos[0] * SCALE), (int(pos[1] * SCALE))), int(0.1 * 20))

        font = pygame.font.SysFont(None, 24)
        timestamp_text = font.render(f"Timestamp: {current_timestamp}", True, (0, 0, 0))
        screen.blit(timestamp_text, (10, 10))

        current_timestamp_index = (current_timestamp_index + 1)
    else:
        running = False

    pygame.display.flip()
    clock.tick(FRAME_RATE)

    time.sleep(0.2)

pygame.quit()