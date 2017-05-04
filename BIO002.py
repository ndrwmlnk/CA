import math, sys, random, time, pickle

import pygame
from pygame.locals import *
from pygame.color import *
import pymunk as pm
from pymunk import Vec2d

class Object(object):
    pass

def draw_collision(arbiter, space, data):
    if True: # ndrw trace attention objects
        for ob in arbiter.shapes:
            if 'attention' in dir(ob):
                if space.time > ob.attention.time[-1]:
                    ob.attention.time.append(space.time)
                    ob.attention.number.append(1)

                    x = arbiter.contact_point_set.normal.x
                    y = arbiter.contact_point_set.normal.y
                    vel_x = ob.body.velocity.x
                    vel_y = ob.body.velocity.y
                    vel_angle_degrees = ob.body.velocity.angle_degrees
                    vel_length = ob.body.velocity.length

                    if y >= 0:
                        val_angle = math.acos(x/math.sqrt(x**2 + y**2))/math.pi*180
                    else:
                        val_angle = 360-math.acos(x / math.sqrt(x ** 2 + y ** 2)) / math.pi * 180

                    ob.attention.impulces.append([[x, y, val_angle, arbiter.contact_point_set.normal.angle_degrees, arbiter.contact_point_set.normal.angle, arbiter.contact_point_set.points[0].distance]]) # [ x,y,angle_degrees_x_y, angle_degrees, angle, distance ]
                    ob.attention.velocity.append([vel_x, vel_y, vel_angle_degrees, vel_length])  # [ x,y,angle_degrees_x_y, angle_degrees, angle, distance ]
                else: # two and more collision points
                    ob.attention.number[-1] += ob.attention.number[-1]
                    x = arbiter.contact_point_set.normal.x
                    y = arbiter.contact_point_set.normal.y
                    if y >= 0:
                        val_angle = math.acos(x/math.sqrt(x**2 + y**2))/math.pi*180
                    else:
                        val_angle = 360-math.acos(x / math.sqrt(x ** 2 + y ** 2)) / math.pi * 180
                    ob.attention.impulces[-1].append([x, y, val_angle, arbiter.contact_point_set.normal.angle_degrees, arbiter.contact_point_set.normal.angle, arbiter.contact_point_set.points[0].distance])  # [ x,y,angle_degrees_x_y, angle_degrees, angle, distance ]

    for c in arbiter.contact_point_set.points:
        r = max( 3, abs(c.distance*2) )
        r = int(r)
        p = tuple(map(int, c.point_a))
        pygame.draw.circle(data["surface"], THECOLORS["red"], p, r, 0)


def draw_collisionG(arbiter, space, data):
    print('- - - - - - - draw_collisionG - - - - - - - -')
    for ob in arbiter.shapes:
        if 'attention' in dir(ob) and 'sim_vel_tmp' in dir(ob):
            if 'solution_vel' in dir(ob.attention):
                ob.attention.solution_vel.append(ob.sim_vel_tmp)
            else:
                ob.attention.solution_vel = [ob.sim_vel_tmp]
# def ipse_search(obj1,obj2,command):
#     objs = [obj1,obj2]
#     for obj in objs:
#         if not obj.STATIC:
#             for trajectories in obj.trajectories:
#                 pass

def main():
    
    global contact
    global shape_to_remove


    ##ndrw
    trajectories = []
    tr_out = range(0, -601, -10)
    tr = [tr_out, [0] * len(tr_out)]
    tr_out = [list(i) for i in zip(*tr)]
    tr_in = range(0, 601, 10)
    tr = [[0] * len(tr_in), tr_in]
    tr_in = [list(i) for i in zip(*tr)]
    trajectories.append([tr_out,tr_in,[20,-20],[0,1],[0]]) # output trajectory, input trajectory, collision point, impuls direction, context
    trajectories.append([tr_in, [], [], [],[1]])  # output trajectory, input trajectory, collision point, impuls direction, context

        
    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    clock = pygame.time.Clock()
    running = True
    
    ### Physics stuff
    space = pm.Space()
    space.gravity = (0.0, 1000.0) # 900
    
    ## Balls
    balls = []
       
    ### walls
    static_body = pm.Body(body_type = pm.Body.STATIC)
    s1 = pm.Segment(static_body, (200.0, 400.0), (400.0, 400.0), 0.0)
    s1._set_friction(1)
    s2 = pm.Segment(static_body, (400.0, 400.0), (400.0, 300.0), 0.0)
    s3 = pm.Segment(static_body, (200.0, 400.0), (200.0, 300.0), 0.0)
    static_lines = [s1,s2,s3]
    space.add(static_lines)

    trg_next_ball = False
    trg_restart = False
    trg_simulation_newBatch = False
    trg_simulation_newIteration = False
    trg_simulation_newIteration_count = 0
    trg_K_v = False
    trg_simulation_run_wait = False
    sim_vel = [(0.0, -500.0),(-500.0, -500.0),(-500.0, 0.0),(-500.0, 500.0),(0.0, 500.0),(500.0, 500.0),(500.0, 0.0),(500.0, -500.0)]
    trg_Pause = False
    trg_new_ball_x = False
    trg_solution_search = False
    trg_solution_newBall = False

    ch01 = space.add_collision_handler(0, 1)
    ch01.data["surface"] = screen
    ch01.post_solve = draw_collision

    ch11 = space.add_collision_handler(1, 1)
    ch11.data["surface"] = screen
    ch11.post_solve = draw_collision

    ch20 = space.add_collision_handler(2, 0)
    ch20.data["surface"] = screen
    ch20.post_solve = draw_collision

    ch21 = space.add_collision_handler(2, 1)
    ch21.data["surface"] = screen
    ch21.post_solve = draw_collision

    ch25 = space.add_collision_handler(2, 5)
    ch25.data["surface"] = screen
    ch25.post_solve = draw_collisionG


    # add goal object
    mass = 1000
    verticies = [(0,0),(10,0),(10,-100),(0,-100)]
    inertia = pm.moment_for_poly(mass,verticies)
    body = pm.Body(mass, inertia)
    body.position = 200, 400
    shapeGoal = pm.Poly(body,verticies)
    shapeGoal._set_friction(1)
    shapeGoal._set_collision_type(5)
    space.add(body, shapeGoal)
    #balls.append(shapeGoal)

    t10 = time.time()
    space.time = []

    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                running = False
            elif event.type == KEYDOWN and event.key == K_a:  # ndrw
                attention = Object()
                attention.time = [0]
                attention.number = [0]
                attention.impulces = [[]]
                attention.velocity = [[]]
                ball.attention = attention
                shape._set_collision_type(2)
                print('K_a')
                trg_Pause = False
            elif event.type == KEYDOWN and event.key == K_c:  # ndrw
                #deepCopy = copy.deepcopy(space.bodies[0])
                bodiesCopy = []
                for body in space.bodies:
                    bodiesCopy.append([tuple(map(int, body.position)), tuple(map(int, body.velocity))])
                print('K_c')
            elif event.type == KEYDOWN and event.key == K_d:  # ndrw
                for b in range(0,len(balls)):
                    #space.remove(ball, ball.body)
                    space.remove(balls[0])
                    balls.remove(balls[0])
            elif event.type == KEYDOWN and event.key == K_f:
                pygame.image.save(screen, "contact_and_no_flipy.png")
            elif event.type == KEYDOWN and event.key == K_h:  # ndrw
                shape.body._set_velocity((0.0, -500.0))
            elif event.type == KEYDOWN and event.key == K_i:  # ndrw input value of x coordinate of a new boal
                new_ball_x = raw_input("Enter new ball X coordinate: ")
                trg_new_ball_x = True
            elif event.type == KEYDOWN and event.key == K_n: #ndrw
                trg_next_ball = True
            elif event.type == KEYDOWN and event.key == K_p:
                if trg_Pause:
                    trg_Pause = False
                    print('K_p > False')
                else:
                    trg_Pause = True
                    print('K_p > True')
            elif event.type == KEYDOWN and event.key == K_r: #ndrw
                trg_restart = True
            elif event.type == KEYDOWN and event.key == K_s:  # ndrw
                trg_simulation_newBatch = True
                print('K_s')
            elif event.type == KEYDOWN and event.key == K_v:  # ndrw
                trg_K_v = True
                print('K_v')
            elif event.type == KEYDOWN and event.key == K_w:  # ndrw - write
                print('K_v')
                for ob in space.shapes:
                    if 'attention' in dir(ob):
                        output = open('attention2.pkl','wb')
                        pickle.dump(ob.attention, output, pickle.HIGHEST_PROTOCOL)
                        output.close()
                        print('pickle.dump(ob.attention, output, pickle.HIGHEST_PROTOCOL)')
            elif event.type == KEYDOWN and event.key == K_z:  # ndrw - execute a solution
                trg_solution_search = True
                pygame.event.post(pygame.event.Event(KEYDOWN, {'key':K_a}))
                trg_Pause = True

                ### load collision database
                input = open('attention_db.pkl', 'rb')
                stackA, stackB = pickle.load(input)
                input.close()
                for ob in space.shapes:
                    if 'attention' in dir(ob):
                        ob.attention.stackA = stackA
                        ob.attention.stackB = stackB
                trg_simulation_newBatch = True
        if not trg_Pause:
            if trg_K_v: # paste saved circles after during and after a simulation
                for b in bodiesCopy:
                    mass = 10
                    radius = 25
                    inertia = pm.moment_for_circle(mass, 0, radius, (0, 0))
                    body = pm.Body(mass, inertia)
                    x = random.randint(200, 400)
                    body.position = b[0]
                    shape = pm.Circle(body, radius, (0, 0))
                    shape.body._set_velocity(b[1])
                    shape._set_collision_type(b[2])
                    if len(b) == 4: # position, velocity, collision_type, attention
                        shape.attention = b[3]
                    space.add(body, shape)
                    balls.append(shape)
                trg_K_v = False
            if trg_solution_newBall and trg_solution_search:
                trg_solution_search = False
                trg_solution_newBall = False

                ### insert a new object as a solution
                trg_next_ball = True
                trg_new_ball_x = True
                collisionPoint = (-0.65, 0.76)
                collisionPointAbs = [i * -ball.radius for i in collisionPoint]
                p = tuple(map(int, ball.body.position))
                pX = p[0] + collisionPointAbs[0]  # X coordinate of a collision required in a task
                new_ball_x = str(int(pX))

            if trg_simulation_newBatch:
                bodiesCopy = []
                for shape_tmp in space.shapes:
                    if type(shape_tmp) is pm.Circle:
                        bodiesCopy.append([tuple(map(int, shape_tmp.body.position)), tuple(map(int, shape_tmp.body.velocity)), shape_tmp.collision_type, shape_tmp.attention])
                trg_simulation_newBatch = False
                trg_simulation_newIteration = True
            if trg_simulation_newIteration and not trg_simulation_run_wait:
                trg_simulation_newIteration_count += 1
                if trg_simulation_newIteration_count <= len(sim_vel):
                    shape.body._set_velocity(sim_vel[trg_simulation_newIteration_count-1]) # set velocity vector to an object
                    shape.sim_vel_tmp = sim_vel[trg_simulation_newIteration_count-1]
                    trg_simulation_run_wait = True
                    t0 = time.time()
                else:
                    trg_simulation_newIteration = False
                    trg_simulation_run_wait = False
                    trg_simulation_newIteration_count = 0
                    trg_solution_newBall = True
            if trg_simulation_run_wait:
                if time.time() - t0 > 1.1:
                    trg_simulation_run_wait = False
                    trg_simulation_newIteration = True
                    trg_restart = True
                    trg_K_v = True
            if trg_next_ball:
                trg_next_ball = False
                mass = 10
                radius = 25
                inertia = pm.moment_for_circle(mass, 0, radius, (0,0))
                body = pm.Body(mass, inertia)
                if trg_new_ball_x:
                    x = int(new_ball_x)
                    trg_new_ball_x = False
                else:
                    x = random.randint(200,400)
                body.position = x, 200
                shape = pm.Circle(body, radius, (0,0))
                shape._set_collision_type(1)
                space.add(body, shape)
                balls.append(shape)

            ### Clear screen
            screen.fill(THECOLORS["white"])

            ### Draw stuff
            balls_to_remove = []
            for ball in balls:
                if (ball.body.position.y > 400) or (trg_restart):
                    balls_to_remove.append(ball)
                p = tuple(map(int, ball.body.position))
                pygame.draw.circle(screen, THECOLORS["blue"], p, int(ball.radius), 2)

                if 'line' in dir(ball): #ndrw > save a trajectory in the ball object
                    ball.line.append(list(p))
                else:
                    ball.line = []
            trg_restart = False

            for ball in balls_to_remove:
                space.remove(ball, ball.body)
                balls.remove(ball)

            for line in static_lines:
                body = line.body
                p1 = body.position + line.a.rotated(body.angle)
                p2 = body.position + line.b.rotated(body.angle)
                pygame.draw.lines(screen, THECOLORS["lightgray"], False, [p1,p2])

            for ball in balls:
                if len(ball.line) > 1:
                    pygame.draw.lines(screen, THECOLORS["lightgray"], False, ball.line)

            pGD = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]
            pG = tuple(map(int, shapeGoal.body.position))
            for vert_id in range(0,len(verticies)): # ndrw
                pGD[vert_id] = tuple(map(lambda x, y: x + y, verticies[vert_id], pG))
            pygame.draw.polygon(screen, THECOLORS["red"], pGD, 2) # ndrw draw goal object



            ### Update physics
            fps = 25.0
            dt = 1.0/fps
            space.time = time.time()-t10
            for x in range(1):
                space.step(dt)

            ### Flip screen
            pygame.display.flip()
            clock.tick(fps)
            pygame.display.set_caption("fps: " + str(clock.get_fps()))
        
if __name__ == '__main__':
    sys.exit(main())
