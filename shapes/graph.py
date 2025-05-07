import pygame
import json
import time
import os

pygame.init()
SCALE = 20
FRAME_RATE = 10
SCREEN_SIZE = (800, 600)
screen = pygame.display.set_mode(SCREEN_SIZE)
pygame.display.set_caption("Timestamp Animation")
clock = pygame.time.Clock()

DATA_DIR = "letters"  # Replace with your directory path

# Get all files in the directory (filtering out non-JSON if needed)
files = [f for f in os.listdir(DATA_DIR) if os.path.isfile(os.path.join(DATA_DIR, f))]

for file_name in files:
    full_path = os.path.join(DATA_DIR, file_name)
    print(f"Processing: {file_name}")

    with open(full_path, "r") as file:
        data = json.load(file)

    timestamps = sorted([int(k) for k in data.keys()])
    current_timestamp_index = 0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        screen.fill((255, 255, 255))

        if timestamps and current_timestamp_index < len(timestamps):
            current_timestamp = timestamps[current_timestamp_index]
            sim_record = data[str(current_timestamp)]

            for pos in sim_record:
                pygame.draw.circle(screen, (0, 0, 255), (int(pos[0] * SCALE), int(pos[1] * SCALE)), int(0.1 * SCALE))

            font = pygame.font.SysFont(None, 24)
            timestamp_text = font.render(f"{file_name} - Timestamp: {current_timestamp}", True, (0, 0, 0))
            screen.blit(timestamp_text, (10, 10))

            current_timestamp_index += 1
        else:
            running = False

        pygame.display.flip()
        clock.tick(FRAME_RATE)
        time.sleep(0.2)

pygame.quit()
