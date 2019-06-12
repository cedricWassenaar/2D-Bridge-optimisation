import sys
import pygame
from pygame.locals import *


WHITE = 255, 255, 255
GREEN = 0, 255, 0
BLACK = 0, 0, 0
BLUE = 0, 0, 255
RED = 255, 0, 0
PURPLE = 128, 0, 128
ORANGE = 255, 165, 0


class Construction(object):
    def __init__(self, name: str, width, height):
        self.size = self.width, self.height = width, height
        self.screen = pygame.display.set_mode(self.size)
        self.clock = pygame.time.Clock()
        pygame.display.set_caption(name)
        pygame.init()

    def draw_beam(self, name: str, pos1, pos2, load, size=2):
        pygame.draw.line(self.screen, WHITE, pos1, pos2, size)
        self.add_text((pos1+pos2) / (2, 2) + (-15, 15), name, clr=BLUE)
        self.add_text((pos1 + pos2) / (2, 2) + (15, -15), "{0:.1f} N".format(round(load, 3)), clr=BLUE)

    def draw_node(self, name: str, pos, size=10):
        pygame.draw.circle(self.screen, WHITE, (int(pos[0]), int(pos[1])), size, 0)  # filled
        self.add_text(pos + (-35, 10), name, clr=RED)

    def draw_force(self, name: str, pos, force, size=10):
        pos2 = pos+((0.1, -0.1)*force)
        pygame.draw.line(self.screen, RED, (int(pos[0]), int(pos[1])), (int(pos2[0]), int(pos2[1])), size)
        self.add_text((pos+pos2) / 2 - (25, -25), name + "=" + str(force), clr=ORANGE)

    def draw_constraint_x(self, name: str, pos, size=7):
        pos2 = pos-((0.2, 0) * pos)
        pygame.draw.line(self.screen, GREEN, pos, pos2, size)
        self.add_text(pos2-(0, 25), name, clr=GREEN)

    def draw_constraint_y(self, name: str, pos, size=7):
        pos2 = pos - ((0, 0.1) * pos)
        pygame.draw.line(self.screen, GREEN, pos, pos2, size)
        self.add_text(pos2-(0, 25), name, clr=GREEN)

    def draw_editable(self, pos, size=7):
        pygame.draw.circle(self.screen, PURPLE, (int(pos[0]), int(pos[1])), size, 0)  # filled

    def add_text(self, pos, text: str, clr=GREEN, size=24):
        font = pygame.font.Font(None, size)
        text_img = font.render(text, 1, clr)
        self.screen.blit(text_img, (int(pos[0]), int(pos[1])))

    def hold(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.display.quit()
                sys.exit(0)
            if event.type == KEYDOWN and event.key == K_ESCAPE:
                pygame.display.quit()
                sys.exit(0)
        self.clock.tick(100)

    def show(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.display.quit()
                sys.exit(0)
            if event.type == KEYDOWN and event.key == K_ESCAPE:
                pygame.display.quit()
                sys.exit(0)
        pygame.display.update()
        self.screen.fill((0, 0, 0))


if __name__ == "__main__":
    G = Construction("Test", 720, 480)
    G.show()
