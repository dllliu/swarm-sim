import pygame

class MaskCreator:
    def __init__(self, maxX, maxY):
        self.maxX = maxX
        self.maxY = maxY

    def create_mask_from_polygon(self, polygon_points):
        temp_surface = pygame.Surface((self.maxX, self.maxY), pygame.SRCALPHA)
        temp_surface.fill((0, 0, 0, 0))
        pygame.draw.polygon(temp_surface, (255, 255, 255, 255), polygon_points)
        return pygame.mask.from_surface(temp_surface)

    def create_mask_from_donut(self, center=(400, 400), outer_radius=200, inner_radius=100):
        temp_surface = pygame.Surface((self.maxX, self.maxY), pygame.SRCALPHA)
        temp_surface.fill((0, 0, 0, 0))
        pygame.draw.circle(temp_surface, (255, 255, 255, 255), center, outer_radius)
        pygame.draw.circle(temp_surface, (0, 0, 0, 0), center, inner_radius)
        return pygame.mask.from_surface(temp_surface)

    def create_mask_from_circle(self, center=(400, 400), radius=200):
        temp_surface = pygame.Surface((self.maxX, self.maxY), pygame.SRCALPHA)
        temp_surface.fill((0, 0, 0, 0))
        pygame.draw.circle(temp_surface, (255, 255, 255, 255), center, radius)
        return pygame.mask.from_surface(temp_surface)

    def create_mask_from_ellipse(self, rect):
        temp_surface = pygame.Surface((self.maxX, self.maxY), pygame.SRCALPHA)
        temp_surface.fill((0, 0, 0, 0))
        pygame.draw.ellipse(temp_surface, (255, 255, 255, 255), rect)
        return pygame.mask.from_surface(temp_surface)

    def create_mask_from_hollow_ellipse(self, rect, boundary_width):
        temp_surface = pygame.Surface((self.maxX, self.maxY), pygame.SRCALPHA)
        temp_surface.fill((0, 0, 0, 0))
        pygame.draw.ellipse(temp_surface, (255, 255, 255, 255), rect, boundary_width)
        return pygame.mask.from_surface(temp_surface)

    def create_mask_from_rect(self, rect):
        temp_surface = pygame.Surface((self.maxX, self.maxY), pygame.SRCALPHA)
        temp_surface.fill((0, 0, 0, 0))
        pygame.draw.rect(temp_surface, (255, 255, 255, 255), rect)
        return pygame.mask.from_surface(temp_surface)

    def create_mask_from_line(self, start_pos, end_pos, width=60):
        temp_surface = pygame.Surface((self.maxX, self.maxY), pygame.SRCALPHA)
        temp_surface.fill((0, 0, 0, 0))
        pygame.draw.line(temp_surface, (255, 255, 255, 255), start_pos, end_pos, width)
        return pygame.mask.from_surface(temp_surface)

    def create_mask_from_lines(self, points, closed=False, width=60):
        temp_surface = pygame.Surface((self.maxX, self.maxY), pygame.SRCALPHA)
        temp_surface.fill((0, 0, 0, 0))
        pygame.draw.lines(temp_surface, (255, 255, 255, 255), closed, points, width)
        return pygame.mask.from_surface(temp_surface)

    def create_mask_from_letter(self, letter: str, font_size=300):
        temp_surface = pygame.Surface((self.maxX, self.maxY), pygame.SRCALPHA)
        temp_surface.fill((0, 0, 0, 0))  # Transparent

        font = pygame.font.SysFont('Arial', font_size, bold=True)
        text = font.render(letter, True, (255, 255, 255))

        # Center the text
        text_rect = text.get_rect(center=(self.maxX // 2, self.maxY // 2))
        temp_surface.blit(text, text_rect)

        return pygame.mask.from_surface(temp_surface)

