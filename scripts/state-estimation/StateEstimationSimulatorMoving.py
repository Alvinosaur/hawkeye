import pygame
import random
import numpy as np
import StateEstimation as se
import math

def distance(x, y, x2, y2):
    return math.sqrt((x - x2)**2 + (y - y2)**2)

def get_center(width, height):
    return (width//2, height//2)

# Represents "more human" motion of a target
class HumanMotion(object):

    def __init__(self, t_step, limit):
        self.cur_t = 0
        self.t_step = t_step
        self.pos = [0, 0]
        self.vel = [0, 0]
        self.acc = [0, 0]
        self.acc_std = limit / 30
        self.vel_limit = limit  / 2  # Max speed, can travel half the frame in a second
        self.acc_limit = limit / 6

    def get_pos(self):
        return tuple(self.pos)

    def get_vel(self):
        return tuple(self.vel)

    def get_acc(self):
        return tuple(self.acc)

    def incr(self):

        # Randomly increment acceleration
        acc_change_x = np.random.normal(0, self.acc_std)
        acc_change_y = np.random.normal(0, self.acc_std)
        self.acc[0] += acc_change_x
        self.acc[1] += acc_change_y
        for i in range(2):
            if self.acc[i] > self.acc_limit:
                self.acc[i] = self.acc_limit
            elif self.acc[i] < -self.acc_limit:
                self.acc[i] = -self.acc_limit

        # Change v and p accordingly
        self.vel[0] += self.acc[0] * self.t_step
        self.vel[1] += self.acc[1] * self.t_step
        for i in range(2):
            if self.vel[i] > self.vel_limit:
                self.vel[i] = self.vel_limit
            elif self.vel[i] < -self.vel_limit:
                self.vel[i] = -self.vel_limit
        self.pos[0] += self.vel[0] * self.t_step
        self.pos[1] += self.vel[1] * self.t_step

class Observations(object):

    def __init__(self, motion, std, bogus_prob):
        self.motion = motion
        self.std = std
        self.bogus_prob = bogus_prob

    def get_observation(self):
        # Add random noise with respect to standard deviation
        (x_pos, y_pos) = self.motion.get_pos()
        (obs_x, obs_y) = (np.random.normal(x_pos, self.std), np.random.normal(y_pos, self.std))

        # With small probability, give a completely bogus point
        multiplier = 10
        p = random.uniform(0, 1)
        if (p < self.bogus_prob):
            return (obs_x + random.choice([-1, 1]) * self.std * multiplier, 
                    obs_y + random.choice([-1, 1]) * self.std * multiplier)

        return (obs_x, obs_y)

class PygameGame(object):

    def init(self):
        pygame.font.init()
        self.font = pygame.font.SysFont('Verdana', round(self.dim / 40))
        self.font2 = pygame.font.SysFont('Impact', round(self.dim / 10))

        # Flags
        self.draw_actual = True
        self.draw_observed = False
        self.draw_predicted = False
        self.draw_drone = True
        self.centered_to = "Drone"

        # Parameters for observations
        self.std = self.dim // 30
        self.bogus_prob = 0
        
        # Start at the "end"
        self.motion = None
        self.ended = True

    def initSimulation(self):
        self.ended = False

        # Metrics
        self.frame_count = 0
        self.erroneous_frames = 0
        self.window_count = 0
        self.erroneous_windows = 0
        self.window_x = []
        self.window_y = []
        self.error_dists = []
        self.track_limit = (1 / 3) / 2
        self.stab_limit = 1 / 6

        # Initialize random target motion
        self.motion = HumanMotion(1 / self.fps, self.dim)

        # Initialize observations
        self.observations = Observations(self.motion, self.std, self.bogus_prob)

        # Initialize estimator
        p_init = self.motion.get_pos()
        v_init = self.motion.get_vel()
        a_init = self.motion.get_acc()
        acc_std = self.dim / 6 # TODO: TUNE THIS PARAMETER
        self.estimator = se.KalmanEstimator(self.fps, self.std, acc_std / 4, (0, 0), (0, 0), (0, 0))

        # Initialize drone speed
        (self.drone_x, self.drone_y) = (0, 0)
        (self.drone_vx, self.drone_vy) = (0, 0)
    
    def mousePressed(self, x, y):
        pass

    def mouseReleased(self, x, y):
        pass

    def mouseMotion(self, x, y):
        pass

    def mouseDrag(self, x, y):
        pass

    def keyPressed(self, keyCode, modifier):
        pass

    def keyReleased(self, keyCode, modifier):
        if keyCode == pygame.K_SPACE and self.ended:
            self.initSimulation()
        elif keyCode == pygame.K_UP and self.ended:
            self.std += self.dim // 600
        elif keyCode == pygame.K_DOWN and self.ended:
            self.std -= self.dim // 600
            if self.std < 0:
                self.std = 0
        elif keyCode == pygame.K_LEFT and self.ended:
            self.bogus_prob -= 0.01
            if self.bogus_prob < 0:
                self.bogus_prob = 0
        elif keyCode == pygame.K_RIGHT and self.ended:
            self.bogus_prob += 0.01
        elif keyCode == pygame.K_a:
            self.draw_actual = not self.draw_actual
        elif keyCode == pygame.K_o:
            self.draw_observed = not self.draw_observed
        elif keyCode == pygame.K_p:
            self.draw_predicted = not self.draw_predicted
        elif keyCode == pygame.K_d:
            self.draw_drone = not self.draw_drone
        elif keyCode == pygame.K_s and not self.ended:
            if self.centered_to == "Drone":
                self.centered_to = "Target"
            else:
                self.centered_to = "Drone"
        elif keyCode == pygame.K_q and not self.ended:
            self.ended = True

    def timerFired(self, dt):
        if not self.ended:
            self.frame_count += 1

            # Move drone
            self.drone_x += self.drone_vx / self.fps
            self.drone_y += self.drone_vy / self.fps

            # Move target
            self.motion.incr()

            # Make an observation
            (self.obs_x, self.obs_y) = self.observations.get_observation()

            # Add observation to estimator
            self.estimator.observe(self.obs_x, self.obs_y)

            # Predict target's position, velocity
            (self.pred_x, self.pred_y) = self.estimator.predict()
            (self.drone_vx, self.drone_vy) = self.estimator.get_vel()

            # Calculate metrics
            (x, y) = self.motion.get_pos()
            if abs(x - self.drone_x) > self.width * self.track_limit or abs(y - self.drone_y) > self.height * self.track_limit:
                self.erroneous_frames += 1
            self.window_x.append(x - self.drone_x)
            self.window_y.append(y - self.drone_y)
            self.error_dists.append(distance(x, y, self.drone_x, self.drone_y))
            if self.frame_count % self.fps == 0:
                std_x = np.std(self.window_x)
                std_y = np.std(self.window_y)
                if std_x > self.width * self.stab_limit or std_y > self.stab_limit * self.height:
                    self.erroneous_windows += 1
                self.window_count += 1
                self.window_x, self.window_y = [], []

    # Prints "lines" so you can get a sense of how fast we're moving
    def display_background(self, screen, centered_to_x, centered_to_y, num_segments):
        vertical_dist = round(self.width / num_segments)
        horizontal_dist = round(self.height / num_segments)
        d_x = centered_to_x % vertical_dist
        d_y = centered_to_y % horizontal_dist
        for i in range(num_segments + 1):
            pygame.draw.line(screen, (200, 200, 200), (vertical_dist * i - d_x, 0), (vertical_dist * i - d_x, self.height), width = 2)
            pygame.draw.line(screen, (200, 200, 200), (0, horizontal_dist * i - d_y), (self.width, horizontal_dist * i - d_y), width = 2)

    # Converts points from "absolute" locations (where target starts at (0, 0))
    # to locations relative to some point in space
    def convert(self, x, y, centered_to_x, centered_to_y):
        d_x = x - centered_to_x
        d_y = y - centered_to_y
        (c_x, c_y) = get_center(self.width, self.height)
        return (d_x + c_x, d_y + c_y)

    def redrawAll(self, screen): 
        if self.motion is not None:  
            # Get the coordinates of the target you are centering screen to
            if (self.centered_to == "Drone"):
                (c_x, c_y) = (self.drone_x, self.drone_y)
            else:
                (c_x, c_y) = self.motion.get_pos()

            # Display lines for motion guidance
            self.display_background(screen, c_x, c_y, 4)

            # Target position
            if self.draw_actual:
                (x, y) = self.motion.get_pos()
                (x_c, y_c) = self.convert(x, y, c_x, c_y)
                pygame.draw.circle(screen, (0, 0, 0), (x_c, y_c), self.dim / 150)
                if x_c < 0 or x_c > self.width or y_c < 0 or y_c > self.height:
                    lost_surf = self.font2.render("TARGET LOST", False, (0, 0, 0))
                    screen.blit(lost_surf, (self.width/12, self.height/12))

            # Observed target position
            if self.draw_observed:
                (x_c, y_c) = self.convert(self.obs_x, self.obs_y, c_x, c_y)
                pygame.draw.circle(screen, (255, 0, 0), (x_c, y_c), self.dim / 150)

            # Predicted target position
            if self.draw_predicted:
                (x_c, y_c) = self.convert(self.pred_x, self.pred_y, c_x, c_y)
                pygame.draw.circle(screen, (0, 0, 255), (x_c, y_c), self.dim / 150)

            # Drone position
            if self.draw_drone:
                (x_c, y_c) = self.convert(self.drone_x, self.drone_y, c_x, c_y)
                pygame.draw.circle(screen, (0, 255, 0), (x_c, y_c), self.dim / 150)

        if self.motion is not None:
            if not self.ended:
                # Print velocity, acceleration
                (x_vel, y_vel) = self.motion.get_vel()
                (x_acc, y_acc) = self.motion.get_acc()
                vel_text = ("X Vel: %0.1f, Y Vel: %0.1f" % (x_vel, y_vel))
                acc_text = ("X Acc: %0.1f, Y Acc: %0.1f" % (x_acc, y_acc))
                vel_surf = self.font.render(vel_text, False, (0, 0, 0))
                acc_surf = self.font.render(acc_text, False, (0, 0, 0))
                screen.blit(vel_surf,(self.width/12, self.height/12))
                screen.blit(acc_surf,(self.width/12, 3*self.height/24))
            else:
                # Metrics
                track_text = ("Tracking Fail: %d / %d frames" % (self.erroneous_frames, self.frame_count))
                stab_text = ("Stability Fail: %d / %d windows" % (self.erroneous_windows, self.window_count))
                error_text = ("Aver. Dist. to Target: %0.2f px" % np.mean(self.error_dists))
                track_surf = self.font.render(track_text, False, (0, 0, 0))
                stab_surf = self.font.render(stab_text, False, (0, 0, 0))
                error_surf = self.font.render(error_text, False, (0, 0, 0))
                screen.blit(track_surf,(self.width/24, self.height/12))
                screen.blit(stab_surf,(self.width/24, 3*self.height/24))
                screen.blit(error_surf,(self.width/24, self.height/6))

        # Print press space to start
        if self.ended:
            restart_text = "Press SPACE to start"
            if self.motion is not None:
                restart_text = "Press SPACE to restart"
            restart_surf = self.font.render(restart_text, False, (0, 0, 0))
            screen.blit(restart_surf,(self.width/12, 11*self.height/12))

        # Toggles
        if self.motion is not None:
            act_surf = self.font.render("Press A to toggle actual pos", False, (0, 0, 0))
            obs_surf = self.font.render("Press O to toggle observed pos", False, (255, 0, 0))
            pred_surf = self.font.render("Press P to toggle predicted pos", False, (0, 0, 255))
            drone_surf = self.font.render("Press D to toggle drone pos", False, (0, 255, 0))
            screen.blit(act_surf,(7*self.width/12, self.height/12))
            screen.blit(obs_surf,(7*self.width/12, 3*self.height/24))
            screen.blit(pred_surf,(7*self.width/12, self.height/6))
            screen.blit(drone_surf,(7*self.width/12, 5*self.height/24))

        # In Motion Toggles
        if not self.ended:
            if self.centered_to == "Drone":
                switch_to = "target"
            else:
                switch_to = "drone"
            switch_surf = self.font.render("Press S to switch to " + switch_to + " POV", False, (0, 0, 0))
            quit_surf = self.font.render("Press Q to end current run", False, (0, 0, 0))
            screen.blit(quit_surf,(self.width/12, 11*self.height/12))
            screen.blit(switch_surf,(7*self.width/12, 11*self.height/12))

        # Parameters
        if self.ended:
            std_text = ("Obs. Std. Dev: %d" % self.std)
            bogus_text = ("Bogus Prob: %0.2f" % self.bogus_prob)
            std_surf = self.font.render(std_text, False, (0, 0, 0))
            bogus_surf = self.font.render(bogus_text, False, (0, 0, 0))
            arrows_surf = self.font.render("(Use arrows to change)", False, (0, 0, 0))
            screen.blit(std_surf,(8*self.width/12, 10*self.height/12))
            screen.blit(bogus_surf,(8*self.width/12, 21*self.height/24))
            screen.blit(arrows_surf,(8*self.width/12, 22*self.height/24))

    def isKeyPressed(self, key):
        ''' return whether a specific key is being held '''
        return self._keys.get(key, False)

    def __init__(self, width=600, height=600, fps=30, title="Target State Estimation"):
        self.width = width
        self.height = height
        self.dim = min(self.width, self.height)
        self.fps = fps
        self.title = title
        self.bgColor = (255, 255, 255)
        pygame.init()

    def run(self):

        clock = pygame.time.Clock()
        screen = pygame.display.set_mode((self.width, self.height))
        # set the title of the window
        pygame.display.set_caption(self.title)

        # stores all the keys currently being held down
        self._keys = dict()

        # call game-specific initialization
        self.init()
        playing = True
        while playing:
            time = clock.tick(self.fps)
            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    self.mousePressed(*(event.pos))
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    self.mouseReleased(*(event.pos))
                elif (event.type == pygame.MOUSEMOTION and
                      event.buttons == (0, 0, 0)):
                    self.mouseMotion(*(event.pos))
                elif (event.type == pygame.MOUSEMOTION and
                      event.buttons[0] == 1):
                    self.mouseDrag(*(event.pos))
                elif event.type == pygame.KEYDOWN:
                    self._keys[event.key] = True
                    self.keyPressed(event.key, event.mod)
                elif event.type == pygame.KEYUP:
                    self._keys[event.key] = False
                    self.keyReleased(event.key, event.mod)
                elif event.type == pygame.QUIT:
                    playing = False
            self.timerFired(time)
            screen.fill(self.bgColor)
            self.redrawAll(screen)
            pygame.display.flip()

        pygame.quit()


def main():
    game = PygameGame()
    game.run()

if __name__ == '__main__':
    main()