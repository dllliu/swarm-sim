import pygame

def create_mask_from_polygon(polygon_points, surface_size=(200, 200)):
    surface = pygame.Surface(surface_size, pygame.SRCALPHA)
    surface.fill((0, 0, 0, 0))
    pygame.draw.polygon(surface, (255, 255, 255, 255), polygon_points)
    return pygame.mask.from_surface(surface)

if __name__ == "__main__":
    pygame.init()

    screen = pygame.display.set_mode((200, 200))
    font = pygame.font.SysFont(None, 24)

    running = True
    last_click = None
    in_mask = False

    # Initialize once
    x_1 = 50
    x_2 = 150

    while running:
        screen.fill((30, 30, 30))

        # Update x values each frame
        x_1 = max(0, x_1 - 1)
        x_2 = min(200, x_2 + 1)

        polygon = [(x_1, 50), (x_2, 50), (125, 100), (150, 150), (50, 150), (75, 100)]
        mask = create_mask_from_polygon(polygon)

        # Draw the mask visually
        for x in range(mask.get_size()[0]):
            for y in range(mask.get_size()[1]):
                if mask.get_at((x, y)):
                    screen.set_at((x, y), (0, 255, 0))

        # Draw last clicked point
        if last_click:
            pygame.draw.circle(screen, (255, 0, 0), last_click, 3)
            msg = "Inside" if in_mask else "Outside"
            text_surface = font.render(msg, True, (255, 255, 255))
            screen.blit(text_surface, (10, 10))

        pygame.display.flip()

        # Limit frame rate
        pygame.time.delay(50)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                last_click = pos
                if 0 <= pos[0] < mask.get_size()[0] and 0 <= pos[1] < mask.get_size()[1]:
                    in_mask = mask.get_at(pos)
                else:
                    in_mask = False

    pygame.quit()
