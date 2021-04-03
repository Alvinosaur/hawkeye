import pygame
import random
import numpy as np
import StateEstimation3DAsynch as se
import math

def distance(x, y, x2, y2):
    return math.sqrt((x - x2)**2 + (y - y2)**2)

# Represents "random" motion of a target
class TestMotion(object):

    # Create the motion as a degree 8 polynomial that fits on the screen
    @staticmethod
    def create_polynomial(t_max, t_step, limit):
        coeff_1 = random.uniform(-1/t_max, 1/t_max)
        coeff_2 = random.uniform(-1/(t_max**2), 1/(t_max**3))
        coeff_3 = random.uniform(-1/(t_max**3), 1/(t_max**3))
        coeff_4 = random.uniform(-1/(t_max**4), 1/(t_max**4))
        coeff_5 = random.uniform(-1/(t_max**5), 1/(t_max**5))
        coeff_6 = random.uniform(-1/(t_max**6), 1/(t_max**6))
        coeff_7 = random.uniform(-1/(t_max**7), 1/(t_max**7))
        coeff_8 = random.uniform(-1/(t_max**8), 1/(t_max**8))

        # Scale position 
        maxim, minum = 0, 100000000000
        for t in np.arange(0, t_max + t_step, t_step):
            val = coeff_1 * t + coeff_2 * t**2 + coeff_3 * t**3 + coeff_4 * t**4 + \
                  coeff_5 * t**5 + coeff_6 * t**6 + coeff_7 * t**7 + coeff_8 * t**8
            if val > maxim:
                maxim = val
            if val < minum:
                minum = val
        r = maxim - minum
        coeff_1 *= (5/6) * limit / r
        coeff_2 *= (5/6) * limit / r
        coeff_3 *= (5/6) * limit / r
        coeff_4 *= (5/6) * limit / r
        coeff_5 *= (5/6) * limit / r
        coeff_6 *= (5/6) * limit / r
        coeff_7 *= (5/6) * limit / r
        coeff_8 *= (5/6) * limit / r
        coeff_0 = -(minum * (5/6) * limit / r) + (limit / 12)
        return [coeff_0, coeff_1, coeff_2, coeff_3, coeff_4, coeff_5, coeff_6, coeff_7, coeff_8]

    def __init__(self, x_coeffs, y_coeffs, t_max, t_step):
        self.x_coeffs = x_coeffs
        self.y_coeffs = y_coeffs
        self.t_max = t_max
        self.cur_t = 0
        self.t_step = t_step

    def get_pos(self, t = None):
        if t is None:
            t = self.cur_t
        if t > self.t_max:
            print("Time out of bounds")
            return None
        multiplier = 1
        x_pos, y_pos = 0, 0
        for i in range(len(self.x_coeffs)):
            x_pos += self.x_coeffs[i] * multiplier
            multiplier *= t
        multiplier = 1
        for i in range(len(self.y_coeffs)):
            y_pos += self.y_coeffs[i] * multiplier
            multiplier *= t
        return (x_pos, y_pos)

    def get_vel(self, fps, t = None):
        if t is None:
            t = self.cur_t
        if t > self.t_max:
            print("Time out of bounds")
            return None
        multiplier = 1
        x_vel, y_vel = 0, 0
        for i in range(1, len(self.x_coeffs)):
            x_vel += self.x_coeffs[i] * multiplier * i
            multiplier *= t
        multiplier = 1
        for i in range(1, len(self.y_coeffs)):
            y_vel += self.y_coeffs[i] * multiplier * i
            multiplier *= t
        factor = (1/self.t_step) / fps
        return (x_vel / factor, y_vel / factor)

    def get_acc(self, fps, t = None):
        if t is None:
            t = self.cur_t
        if t > self.t_max:
            print("Time out of bounds")
            return None
        multiplier = 1
        x_acc, y_acc = 0, 0
        for i in range(2, len(self.x_coeffs)):
            x_acc += self.x_coeffs[i] * multiplier * i * (i - 1)
            multiplier *= t
        multiplier = 1
        for i in range(2, len(self.y_coeffs)):
            y_acc += self.y_coeffs[i] * multiplier * i * (i - 1)
            multiplier *= t
        factor = (1/self.t_step) / fps
        return (x_acc / factor**2, y_acc / factor**2)

    def incr(self):
        self.cur_t += self.t_step
        if self.cur_t >= self.t_max:
            self.cur_t = self.t_max
            return True
        return False

    def at_end(self):
        return self.cur_t >= self.t_max


class Observations(object):

    def __init__(self, motion, std, bogus_prob):
        self.motion = motion
        self.std = std
        self.bogus_prob = bogus_prob

    def get_observation(self, drone_x, drone_y, width, height):
        # Add random noise with respect to standard deviation
        (x_pos, y_pos) = self.motion.get_pos()
        (obs_x, obs_y) = (np.random.normal(x_pos, self.std), np.random.normal(y_pos, self.std))

        # With small probability, give a completely bogus point
        p = random.uniform(0, 1)
        if (p < self.bogus_prob):
            return (random.randint(-width // 2, width // 2) + drone_x, 
                    random.randint(-height // 2, height // 2) + drone_y)
        else:
            return (obs_x, obs_y)


class PygameGame(object):

    def init(self):
        pygame.font.init()
        self.font = pygame.font.SysFont('Verdana', round(self.dim / 40))

        # Flags
        self.draw_actual = True
        self.draw_observed = False
        self.draw_predicted = False
        self.draw_drone = True

        # Parameters for observations
        self.std = self.dim // 30
        self.bogus_prob = 0
        
        # Start at the "end"
        self.motion = None
        self.ended = True

    def initSimulation(self):
        self.ended = False
        self.obs_errors = []
        self.pred_errors = []
        self.drone_errors = []
        self.frame_count = 0

        # Initialize random target motion
        t_max = 3
        t_step = 0.02
        x_coeffs = TestMotion.create_polynomial(t_max, t_step, self.width)
        y_coeffs = TestMotion.create_polynomial(t_max, t_step, self.height)
        self.motion = TestMotion(x_coeffs, y_coeffs, t_max, t_step)

        # Initialize observations
        self.observations = Observations(self.motion, self.std, self.bogus_prob)

        # Initialize estimator
        p_init = self.motion.get_pos()
        v_init = self.motion.get_vel(self.fps)
        a_init = self.motion.get_acc(self.fps)
        p_init_ = (p_init[0], p_init[1], 0) # Add dummy z-coordinate
        v_init_ = (v_init[0], v_init[1], 0)
        a_init_ = (a_init[0], a_init[1], 0)
        acc_std = self.dim / 6 # TODO: TUNE THIS PARAMETER
        self.estimator = se.KalmanEstimator(self.dim / 10, self.std, acc_std / 4, p_init_, v_init_, a_init_)

        # Initialize drone position
        (self.drone_x, self.drone_y) = p_init
    
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

    def timerFired(self, dt):
        if not self.ended:
            t = self.frame_count / self.fps

            # Predict target's state
            (self.pred_x, self.pred_y, tmp1, self.drone_vx, self.drone_vy, tmp2, self.drone_ax, self.drone_ay, tmp3) = \
                self.estimator.predict_target_state(t)

            # Move drone
            self.drone_x += self.drone_vx / self.fps
            self.drone_y += self.drone_vy / self.fps

            # Move target
            self.motion.incr()

            if (self.frame_count % 2 == 0):
                # Sample target's motion
                (self.obs_x, self.obs_y) = self.observations.get_observation(self.drone_x, self.drone_y, self.width, self.height)

                # Add sample to estimator
                self.estimator.receive_sample_relative(self.obs_x - self.drone_x, self.obs_y - self.drone_y, 0, t)

            else:
                # Add drone's state to estimator
                self.estimator.receive_drone_state(self.drone_x, self.drone_y, 0, self.drone_vx, self.drone_vy, 0, self.drone_ax, self.drone_ay, 0, t)

            # Calculate error
            (x, y) = self.motion.get_pos()
            self.obs_errors.append(distance(x, y, self.obs_x, self.obs_y))
            self.pred_errors.append(distance(x, y, self.pred_x, self.pred_y))
            self.drone_errors.append(distance(x, y, self.drone_x, self.drone_y))

            # Increment frame
            self.frame_count += 1

        if self.motion is None or self.motion.at_end():
            self.ended = True

    def redrawAll(self, screen):   
        if self.motion is not None:     
            # Target position
            if self.draw_actual:
                (x, y) = self.motion.get_pos()
                pygame.draw.circle(screen, (0, 0, 0), (x, y), self.dim / 150)

            # Observed target position
            if self.draw_observed:
                pygame.draw.circle(screen, (255, 0, 0), (self.obs_x, self.obs_y), self.dim / 150)

            # Predicted target position
            if self.draw_predicted:
                pygame.draw.circle(screen, (0, 0, 255), (self.pred_x, self.pred_y), self.dim / 150)

            # Drone position
            if self.draw_drone:
                pygame.draw.circle(screen, (0, 255, 0), (self.drone_x, self.drone_y), self.dim / 150)

        # Print velocity, acceleration
        if self.motion is not None:
            if not self.ended:
                (x_vel, y_vel) = self.motion.get_vel(self.fps)
                (x_acc, y_acc) = self.motion.get_acc(self.fps)
                vel_text = ("X Vel: %0.1f, Y Vel: %0.1f" % (x_vel, y_vel))
                acc_text = ("X Acc: %0.1f, Y Acc: %0.1f" % (x_acc, y_acc))
                vel_surf = self.font.render(vel_text, False, (0, 0, 0))
                acc_surf = self.font.render(acc_text, False, (0, 0, 0))
                screen.blit(vel_surf,(self.width/12, self.height/12))
                screen.blit(acc_surf,(self.width/12, 3*self.height/24))
            else:
                obs_error = np.mean(self.obs_errors)
                pred_error = np.mean(self.pred_errors)
                drone_error = np.mean(self.drone_errors)
                obs_text = ("Average observ. error: %0.2f px" % obs_error)
                pred_text = ("Average pred. error: %0.2f px" % pred_error)
                drone_text = ("Average drone error: %0.2f px" % drone_error)
                obs_surf = self.font.render(obs_text, False, (255, 0, 0))
                pred_surf = self.font.render(pred_text, False, (0, 0, 255))
                drone_surf = self.font.render(drone_text, False, (0, 255, 0))
                screen.blit(obs_surf,(self.width/24, self.height/12))
                screen.blit(pred_surf,(self.width/24, 3*self.height/24))
                #screen.blit(drone_surf,(self.width/24, self.height/6))

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
