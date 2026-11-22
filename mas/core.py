from collections import deque
from collections import defaultdict
import json
import random
import math
from typing import Optional
Position = tuple[int, int]
class Settings:
    def __init__(self, width=18, height=18, n_robots=6, n_stations=2,
                 n_orders=30, arrivals=80, max_ticks=750, seed=42,
                 battery_capacity=220, energy_per_step=1, reserve=5,
                 charging_rate=9, coordination_delay=1, obstacles=0.10,
                 scenario='medium'):
        self.width = width
        self.height = height
        self.n_robots = n_robots
        self.n_stations = n_stations
        self.n_orders = n_orders
        self.arrivals = arrivals
        self.max_ticks = max_ticks
        self.seed = seed
        self.battery_capacity = battery_capacity
        self.energy_per_step = energy_per_step
        self.reserve = reserve
        self.charging_rate = charging_rate
        self.coordination_delay = coordination_delay
        self.obstacles = obstacles
        self.scenario = scenario
    def validate(self):
        if min(self.width, self.height) < 5 or min(self.n_robots,self.n_stations,self.n_orders) < 1:
            raise ValueError('Map dimension >= 5, and agent/order counts >= 1')
        if self.obstacles < 0 or self.obstacles > .35 or self.coordination_delay < 0:
            raise ValueError('invalid obstacles or coordination_delay')
        if min(self.battery_capacity,self.energy_per_step,self.charging_rate) <= 0 or self.reserve < 0:
            raise ValueError('invalid energy settings')
        if self.battery_capacity <= self.reserve + self.energy_per_step:
            raise ValueError('Battery capacity must exceed reserve plus one movement step')
        if self.max_ticks < 1 or self.arrivals < 1:
            raise ValueError('invalid duration')
class Immutable:
    def __setattr__(self, key, value):
        if getattr(self, "_locked", False):
            raise AttributeError("initial scenario data cannot be changed")
        object.__setattr__(self, key, value)
    def _freeze(self):
        object.__setattr__(self, "_locked", True)
class OrderSpec(Immutable):
    def __init__(self, id: str, pickup: Position, dropoff: Position, release: int):
        self.id = id
        self.pickup = pickup
        self.dropoff = dropoff
        self.release = release
        self._freeze()
class Order:
    def __init__(self, spec: OrderSpec, status='NEW', robot: Optional[str]=None,
                 announced: Optional[int]=None, assigned_at: Optional[int]=None,
                 picked_at: Optional[int]=None, completed_at: Optional[int]=None,
                 attempts=0):
        self.spec = spec
        self.status = status
        self.robot = robot
        self.announced = announced
        self.assigned_at = assigned_at
        self.picked_at = picked_at
        self.completed_at = completed_at
        self.attempts = attempts
class Message:
    def __init__(self, id: int, kind: str, sender: str, receiver: str,
                 task_id: Optional[str], sent_at: int, delivered_at: int,
                 payload=None):
        self.id = id
        self.kind = kind
        self.sender = sender
        self.receiver = receiver
        self.task_id = task_id
        self.sent_at = sent_at
        self.delivered_at = delivered_at
        self.payload = {} if payload is None else payload
class Courier:
    def __init__(self, id: str, position: Position, battery: int, capacity: int,
                 state='IDLE', order_id: Optional[str]=None, route=None,
                 target: Optional[Position]=None, pending_award: Optional[str]=None,
                 distance=0, charging_wait=0, charger_id: Optional[str]=None):
        self.id = id
        self.position = position
        self.battery = battery
        self.capacity = capacity
        self.state = state
        self.order_id = order_id
        self.route = [] if route is None else route
        self.target = target
        self.pending_award = pending_award
        self.distance = distance
        self.charging_wait = charging_wait
        self.charger_id = charger_id
    def free(self):
        return self.state == 'IDLE' and self.order_id is None and self.pending_award is None
class Charger:
    def __init__(self, id: str, position: Position, occupant: Optional[str]=None,
                 queue=None):
        self.id = id
        self.position = position
        self.occupant = occupant
        self.queue = [] if queue is None else queue
        self.queued_at = {}
        self.battery_at_request = {}
class Scenario(Immutable):
    def __init__(self, width: int, height: int, walls: frozenset[Position],
                 courier_positions: tuple[Position, ...],
                 charger_positions: tuple[Position, ...],
                 orders: tuple[OrderSpec, ...],
                 initial_battery: tuple[int, ...], seed: int):
        self.width = width
        self.height = height
        self.walls = walls
        self.courier_positions = courier_positions
        self.charger_positions = charger_positions
        self.orders = orders
        self.initial_battery = initial_battery
        self.seed = seed
        self._freeze()
    def __eq__(self, other):
        if not isinstance(other, Scenario):
            return NotImplemented
        return (self.width == other.width
                and self.height == other.height
                and self.walls == other.walls
                and self.courier_positions == other.courier_positions
                and self.charger_positions == other.charger_positions
                and self.initial_battery == other.initial_battery
                and self.seed == other.seed
                and [(o.id, o.pickup, o.dropoff, o.release) for o in self.orders]
                == [(o.id, o.pickup, o.dropoff, o.release) for o in other.orders])
class Grid:
    def __init__(self, scenario: Scenario):
        self.width, self.height, self.walls = scenario.width, scenario.height, scenario.walls
        self.cache = {}
    def neighbors(self, p):
        x,y=p
        for q in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
            if 0<=q[0]<self.width and 0<=q[1]<self.height and q not in self.walls:
                yield q
    def path(self, src: Position, dst: Position):
        if src == dst: return []
        key=(src,dst)
        if key in self.cache: return list(self.cache[key]) if self.cache[key] is not None else None
        queue=deque([src]); prev={src:None}
        while queue:
            p=queue.popleft()
            if p==dst: break
            for q in self.neighbors(p):
                if q not in prev:
                    prev[q]=p; queue.append(q)
        if dst not in prev:
            self.cache[key]=None;return None
        result=[];p=dst
        while p != src:
            result.append(p);p=prev[p]
        result.reverse(); self.cache[key]=tuple(result)
        return result
    def distance(self,a,b):
        p=self.path(a,b)
        return len(p) if p is not None else math.inf
def make_scenario(cfg:Settings):
    cfg.validate()
    rng=random.Random(cfg.seed)
    all_cells=[(x,y) for y in range(cfg.height) for x in range(cfg.width)]
    rng.shuffle(all_cells)
    wall_count=int(cfg.width*cfg.height*cfg.obstacles)
    walls=set(all_cells[:wall_count])
    unvisited=set(all_cells)-walls; components=[]
    while unvisited:
        start=min(unvisited); q=deque([start]);unvisited.remove(start);component=[start]
        while q:
            x,y=q.popleft()
            for p in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                if p in unvisited:
                    unvisited.remove(p);q.append(p);component.append(p)
        components.append(component)
    free=max(components,key=lambda comp:(len(comp),-min(comp)[0],-min(comp)[1]))
    if len(free)<cfg.n_robots+cfg.n_stations+2:
        raise ValueError('not enough reachable cells for agents')
    walls=set(all_cells)-set(free)
    rng.shuffle(free)
    stations=tuple(free[:cfg.n_stations])
    robots=tuple(free[cfg.n_stations:cfg.n_stations+cfg.n_robots])
    available=free[cfg.n_stations+cfg.n_robots:]
    orders=[]
    for i in range(cfg.n_orders):
        p,d=rng.sample(available,2)
        release=round(i*cfg.arrivals/max(1,cfg.n_orders-1))
        orders.append(OrderSpec(f'T{i+1:03d}',p,d,release))
    battery=tuple(cfg.battery_capacity for _ in robots)
    return Scenario(cfg.width,cfg.height,frozenset(walls),robots,stations,tuple(orders),battery,cfg.seed)
class Simulation:
    def __init__(self, cfg: Settings, mode: str,
                 scenario: Optional[Scenario] = None, trace: bool = False,
                 charge_policy: str = 'fifo'):
        if mode not in ('central', 'contract'):
            raise ValueError('mode must be central or contract')
        if charge_policy not in ('fifo', 'priority'):
            raise ValueError('charge_policy must be fifo or priority')
        cfg.validate()
        self.charge_policy = charge_policy
        self.cfg = cfg
        self.mode = mode
        self.scenario = scenario if scenario is not None else make_scenario(cfg)
        if (self.scenario.width, self.scenario.height) != (cfg.width, cfg.height):
            raise ValueError('Scenario dimensions do not match settings')
        self.grid = Grid(self.scenario)
        self.couriers = {
            f'R{i+1}': Courier(f'R{i+1}', pos, b, cfg.battery_capacity)
            for i, (pos, b) in enumerate(zip(self.scenario.courier_positions,
                                             self.scenario.initial_battery))
        }
        self.chargers = {
            f'C{i+1}': Charger(f'C{i+1}', pos)
            for i, pos in enumerate(self.scenario.charger_positions)
        }
        self.orders = {spec.id: Order(spec) for spec in self.scenario.orders}
        self.clock = 0
        self.mailbox = defaultdict(list)
        self.in_flight = []
        self.next_message_id = 1
        self.message_counts = defaultdict(int)
        self.message_bytes = 0
        self.trace_enabled = trace
        self.events = []
        self.rounds = {}
        self.waiting_award = {}
        self.retry_at = defaultdict(int)
        self.completed = 0
        self.assignments = 0
        self.rejections = 0
        self.collisions_avoided = 0
        self.charge_wait_total = 0
        self.total_distance = 0
        self.deadlocks = 0
        self.recoveries = 0
        self.recovery_ticks_total = 0
        self.stall_ticks = defaultdict(int)
        self.recovery_due = {}
        self.charging_waits = []
        self.urgent_waits = []
        self.charging_grants = 0
        self.charging_busy_ticks = 0
        self.max_charger_queue = 0
        self.charger_violations = 0
        self.charger_constraint_checks = 0
    def event(self, type_, **data):
        if self.trace_enabled:
            self.events.append({'tick': self.clock, 'type': type_, **data})
    def send(self, kind, sender, receiver, task_id=None, payload=None):
        body = {} if payload is None else payload
        message = Message(self.next_message_id, kind, sender, receiver,
                          task_id, self.clock,
                          self.clock + self.cfg.coordination_delay + 1, body)
        self.next_message_id += 1
        self.in_flight.append(message)
        self.message_counts[kind] += 1
        self.message_bytes += len(json.dumps({
            'id': message.id, 'type': kind, 'from': sender,
            'to': receiver, 'task': task_id, 'payload': body
        }, ensure_ascii=False).encode('utf-8'))
        self.event('send', kind=kind, sender=sender, receiver=receiver,
                   task=task_id)
    def deliver(self):
        due = [m for m in self.in_flight if m.delivered_at <= self.clock]
        self.in_flight = [m for m in self.in_flight
                          if m.delivered_at > self.clock]
        for message in due:
            self.mailbox[message.receiver].append(message)
    def service_positions(self, station):
        return tuple(self.grid.neighbors(station.position))
    def nearest_service_distance(self, position):
        distances = [self.grid.distance(position, stop)
                     for station in self.chargers.values()
                     for stop in self.service_positions(station)]
        return min(distances, default=math.inf)
    def nearest_charger(self, position):
        return min(self.chargers.values(), key=lambda station:
                   (min((self.grid.distance(position, cell)
                         for cell in self.service_positions(station)),
                        default=math.inf), station.id))
    def charger_stop(self, robot, station):
        occupied = {r.position for r in self.couriers.values()
                    if r.id != robot.id}
        reserved = {r.target for r in self.couriers.values()
                    if r.id != robot.id and r.state == 'TO_CHARGER'}
        options = [cell for cell in self.service_positions(station)
                   if cell not in occupied and cell not in reserved]
        return min(options, key=lambda p: (self.grid.distance(robot.position, p), p)) if options else None
    def energy_required(self, robot, order):
        a = self.grid.distance(robot.position, order.spec.pickup)
        b = self.grid.distance(order.spec.pickup, order.spec.dropoff)
        c = self.nearest_service_distance(order.spec.dropoff)
        if not all(math.isfinite(x) for x in (a, b, c)):
            return None
        base_steps = a + b + c
        headroom = max(self.cfg.reserve, math.ceil(base_steps * 0.20))
        return (a + b, base_steps * self.cfg.energy_per_step + headroom)
    def estimate(self, robot: Courier, order: Order):
        if not robot.free() or order.status not in ('NEW', 'REQUEUED', 'ANNOUNCED'):
            return None
        cost = self.energy_required(robot, order)
        if cost is None:
            return None
        distance, required = cost
        if robot.battery < required:
            return None
        risk = max(0, required + 20 - robot.battery) / max(1, self.cfg.battery_capacity)
        return {'bid': round(distance + 12 * risk, 4),
                'distance': distance, 'required': required}
    def estimate_for_award(self, robot, order):
        if robot.state != 'IDLE' or robot.order_id is not None:
            return False
        cost = self.energy_required(robot, order)
        return cost is not None and robot.battery >= cost[1]
    def award(self, order, robot_id):
        if order.status not in ('NEW', 'REQUEUED', 'ANNOUNCED'):
            return
        robot = self.couriers[robot_id]
        if not robot.free():
            return
        order.status = 'RESERVED'
        order.robot = robot_id
        robot.pending_award = order.spec.id
        self.waiting_award[order.spec.id] = self.clock + 2 * (self.cfg.coordination_delay + 1) + 3
        self.send('TASK_AWARD', 'D', robot_id, order.spec.id)
    def cancel_award(self, order):
        if order.robot and self.couriers[order.robot].pending_award == order.spec.id:
            self.couriers[order.robot].pending_award = None
        order.robot = None
        order.status = 'REQUEUED'
        order.attempts += 1
        self.retry_at[order.spec.id] = self.clock + 2
        self.event('award_cancelled', order=order.spec.id)
    def broker_step(self):
        for message in self.mailbox.pop('D', []):
            if message.kind == 'TASK_OFFER' and message.task_id in self.rounds:
                self.rounds[message.task_id]['offers'][message.sender] = message.payload['bid']
            elif message.kind == 'TASK_REJECT':
                self.rejections += 1
            elif message.kind == 'TASK_ACK' and message.task_id in self.waiting_award:
                robot = self.couriers[message.sender]
                order = self.orders[message.task_id]
                if order.robot == robot.id and order.status == 'RESERVED':
                    if message.payload.get('ok') and self.estimate_for_award(robot, order):
                        order.status = 'IN_PROGRESS'
                        robot.order_id = order.spec.id
                        robot.pending_award = None
                        order.assigned_at = self.clock
                        self.assignments += 1
                        self.set_route(robot, order.spec.pickup, 'TO_PICKUP')
                        self.event('assignment', order=order.spec.id, robot=robot.id)
                    else:
                        self.cancel_award(order)
                self.waiting_award.pop(message.task_id, None)
            elif message.kind == 'TASK_COMPLETE' and message.task_id in self.orders:
                order = self.orders[message.task_id]
                if order.status == 'AWAIT_COMPLETE' and order.robot == message.sender:
                    order.status = 'COMPLETED'
                    order.completed_at = self.clock
                    self.completed += 1
                    self.event('completed', order=order.spec.id,
                               robot=message.sender,
                               latency=self.clock - order.spec.release)
        for order_id, deadline in list(self.waiting_award.items()):
            if self.clock > deadline:
                self.cancel_award(self.orders[order_id])
                self.waiting_award.pop(order_id, None)
        if self.mode == 'contract':
            for order_id, round_ in list(self.rounds.items()):
                if self.clock >= round_['deadline']:
                    order = self.orders[order_id]
                    candidates = []
                    for robot_id, bid in round_['offers'].items():
                        robot = self.couriers[robot_id]
                        if self.estimate(robot, order) is not None:
                            candidates.append((bid, robot_id))
                    if candidates:
                        _, winner = min(candidates)
                        self.award(order, winner)
                    else:
                        order.status = 'REQUEUED'
                        self.retry_at[order_id] = self.clock + max(12, 4 * (self.cfg.coordination_delay + 1))
                    del self.rounds[order_id]
        candidates = sorted((o for o in self.orders.values()
                             if o.spec.release <= self.clock
                             and o.status in ('NEW', 'REQUEUED')
                             and o.spec.id not in self.waiting_award
                             and o.spec.id not in self.rounds
                             and self.retry_at[o.spec.id] <= self.clock),
                            key=lambda o: (o.spec.release, o.spec.id))
        for order in candidates:
            if self.mode == 'central':
                bids = []
                for robot in self.couriers.values():
                    estimate = self.estimate(robot, order)
                    if estimate is not None:
                        bids.append((estimate['bid'], robot.id))
                if not bids:
                    continue
                self.award(order, min(bids)[1])
            else:
                order.status = 'ANNOUNCED'
                order.announced = self.clock
                order.attempts += 1
                self.rounds[order.spec.id] = {
                    'offers': {},
                    'deadline': self.clock + 2 * (self.cfg.coordination_delay + 1) + 1
                }
                for robot_id in self.couriers:
                    self.send('TASK_ANNOUNCE', 'D', robot_id, order.spec.id)
            break  
    def set_route(self, robot, target, state):
        path = self.grid.path(robot.position, target)
        if path is None:
            raise RuntimeError('unreachable target')
        robot.route = path
        robot.target = target
        robot.state = state
    def request_charging(self, robot):
        for station in sorted(self.chargers.values(), key=lambda c:
                              (min((self.grid.distance(robot.position, stop)
                                    for stop in self.service_positions(c)), default=math.inf), c.id)):
            stop = self.charger_stop(robot, station)
            if stop is None:
                continue
            path = self.grid.path(robot.position, stop)
            if path is None or robot.battery < len(path) * self.cfg.energy_per_step:
                continue
            robot.charger_id = station.id
            self.set_route(robot, stop, 'TO_CHARGER')
            return True
        return False
    def emergency_recovery(self, robot, cause):
        if robot.state == 'RECOVERING':
            return
        if robot.order_id is not None:
            order = self.orders[robot.order_id]
            if order.status == 'IN_PROGRESS':
                order.status = 'REQUEUED'
                order.robot = None
                order.assigned_at = None
                order.picked_at = None
                order.attempts += 1
                self.retry_at[order.spec.id] = self.clock + 3
                self.event('task_requeued', order=order.spec.id, cause=cause)
        if robot.pending_award is not None:
            self.cancel_award(self.orders[robot.pending_award])
        for station in self.chargers.values():
            if station.occupant == robot.id:
                station.occupant = None
            if robot.id in station.queue:
                station.queue.remove(robot.id)
            station.queued_at.pop(robot.id, None)
            station.battery_at_request.pop(robot.id, None)
        robot.order_id = None
        robot.pending_award = None
        robot.charger_id = None
        robot.route = []
        robot.target = None
        robot.state = 'RECOVERING'
        self.recovery_due[robot.id] = self.clock + 15
        self.recoveries += 1
        self.recovery_ticks_total += 15
        self.event('emergency_recovery', robot=robot.id, cause=cause)
    def agent_step(self, robot: Courier):
        for message in self.mailbox.pop(robot.id, []):
            if message.kind == 'TASK_ANNOUNCE':
                order = self.orders[message.task_id]
                bid = self.estimate(robot, order)
                if bid is not None:
                    self.send('TASK_OFFER', robot.id, 'D', order.spec.id,
                              {'bid': bid['bid']})
                else:
                    self.send('TASK_REJECT', robot.id, 'D', order.spec.id)
            elif message.kind == 'TASK_AWARD':
                order = self.orders[message.task_id]
                accepted = (robot.pending_award == message.task_id
                            and self.estimate_for_award(robot, order))
                self.send('TASK_ACK', robot.id, 'D', message.task_id,
                          {'ok': bool(accepted)})
            elif message.kind == 'CHARGE_GRANT':
                if robot.state == 'WAIT_CHARGE' and robot.charger_id == message.sender:
                    robot.state = 'CHARGING'
                    robot.route = []
                    self.event('charging', robot=robot.id, station=message.sender)
        if robot.state == 'RECOVERING':
            if self.clock >= self.recovery_due[robot.id]:
                robot.battery = robot.capacity
                robot.state = 'IDLE'
                robot.charger_id = None
                self.event('recovery_completed', robot=robot.id)
            return
        if robot.state == 'IDLE' and robot.pending_award is None:
            threshold = max(self.cfg.battery_capacity * 0.65,
                            self.cfg.reserve + 2 * self.cfg.energy_per_step)
            if robot.battery < threshold:
                if not self.request_charging(robot):
                    if robot.battery < self.cfg.energy_per_step + self.cfg.reserve:
                        self.emergency_recovery(robot, 'no reachable free charging stop')
        if robot.state == 'CHARGING':
            robot.battery = min(robot.capacity,
                                robot.battery + self.cfg.charging_rate)
            if robot.battery == robot.capacity:
                self.send('CHARGE_RELEASE', robot.id, robot.charger_id)
                robot.state = 'IDLE'
                robot.charger_id = None
        elif robot.state == 'WAIT_CHARGE':
            robot.charging_wait += 1
            self.charge_wait_total += 1
    def charging_priority(self, station, robot_id):
        robot = self.couriers[robot_id]
        waiting = self.clock - station.queued_at[robot_id]
        urgency = 100.0 * (1.0 - robot.battery / max(1, robot.capacity))
        return urgency + 0.8 * waiting
    def station_step(self, station: Charger):
        for message in self.mailbox.pop(station.id, []):
            if message.kind == 'CHARGE_REQUEST':
                robot = self.couriers[message.sender]
                if (robot.state == 'WAIT_CHARGE' and robot.charger_id == station.id
                        and robot.id != station.occupant and robot.id not in station.queue):
                    station.queue.append(robot.id)
                    station.queued_at[robot.id] = self.clock
                    station.battery_at_request[robot.id] = robot.battery / max(1, robot.capacity)
                    self.event('charge_queued', station=station.id, robot=robot.id,
                               battery=robot.battery, queue_length=len(station.queue))
            elif message.kind == 'CHARGE_RELEASE':
                if station.occupant == message.sender:
                    station.occupant = None
                    self.event('charge_released', station=station.id, robot=message.sender)
        valid = [rid for rid in station.queue
                 if self.couriers[rid].state == 'WAIT_CHARGE'
                 and self.couriers[rid].charger_id == station.id]
        for rid in set(station.queue) - set(valid):
            station.queued_at.pop(rid, None)
            station.battery_at_request.pop(rid, None)
        station.queue = valid
        self.max_charger_queue = max(self.max_charger_queue, len(station.queue))
        if station.occupant is not None:
            self.charging_busy_ticks += 1
        if station.occupant is None and station.queue:
            if self.charge_policy == 'fifo':
                robot_id = station.queue[0]
            else:
                robot_id = min(station.queue, key=lambda rid:
                               (-self.charging_priority(station, rid),
                                station.queued_at[rid], rid))
            station.queue.remove(robot_id)
            elapsed = self.clock - station.queued_at.pop(robot_id)
            battery_ratio = station.battery_at_request.pop(robot_id)
            self.charging_waits.append(elapsed)
            if battery_ratio <= 0.35:
                self.urgent_waits.append(elapsed)
            station.occupant = robot_id
            self.charging_grants += 1
            self.event('charge_granted', station=station.id, robot=robot_id,
                       wait_ticks=elapsed, policy=self.charge_policy,
                       battery_ratio=round(battery_ratio, 4))
            self.send('CHARGE_GRANT', station.id, robot_id)
    def move_step(self):
        occupied = {r.position: rid for rid, r in self.couriers.items()}
        moving_states = ('TO_PICKUP', 'TO_DROPOFF', 'TO_CHARGER')
        movers = {rid: r for rid, r in self.couriers.items()
                  if r.state in moving_states and r.route}
        for rid, robot in movers.items():
            if robot.route[0] in occupied:
                blocked = set(occupied) - {robot.position}
                queue = deque([robot.position])
                prev = {robot.position: None}
                while queue and robot.target not in prev:
                    loc = queue.popleft()
                    for position in self.grid.neighbors(loc):
                        if position not in blocked and position not in prev:
                            prev[position] = loc
                            queue.append(position)
                if robot.target in prev:
                    path = []
                    position = robot.target
                    while position != robot.position:
                        path.append(position)
                        position = prev[position]
                    robot.route = list(reversed(path))
        proposed = {rid: r.route[0] for rid, r in movers.items() if r.route}
        yield_moves = set()
        proposed_destinations = set(proposed.values())
        for mover_id in sorted(movers):
            mover = movers[mover_id]
            blockers = [occupied.get(mover.target), occupied.get(mover.route[0])]
            for blocker_id in blockers:
                if blocker_id is None or blocker_id == mover_id or blocker_id in proposed:
                    continue
                blocker = self.couriers[blocker_id]
                if blocker.state != 'IDLE' or blocker.pending_award is not None:
                    continue
                if blocker.battery < self.cfg.energy_per_step:
                    self.emergency_recovery(blocker, 'insufficient battery to yield')
                    continue
                free = [p for p in self.grid.neighbors(blocker.position)
                        if p not in occupied and p not in proposed_destinations]
                if free:
                    to_cell = min(free, key=lambda cell:
                                  (-self.grid.distance(cell, mover.target), cell))
                    proposed[blocker_id] = to_cell
                    proposed_destinations.add(to_cell)
                    yield_moves.add(blocker_id)
                    self.event('yield', robot=blocker_id, for_robot=mover_id)
        chosen = {}
        reserved = set()
        ordered = sorted(proposed, key=lambda rid:
                         (int(rid[1:]) + self.clock) % max(1, len(self.couriers)))
        for rid in ordered:
            destination = proposed[rid]
            if destination in reserved or self.couriers[rid].battery < self.cfg.energy_per_step:
                self.collisions_avoided += 1
                continue
            chosen[rid] = destination
            reserved.add(destination)
        changed = True
        while changed:
            changed = False
            for rid, destination in list(chosen.items()):
                occupant = occupied.get(destination)
                if occupant is not None and (occupant not in chosen or
                        chosen.get(occupant) == self.couriers[rid].position):
                    del chosen[rid]
                    self.collisions_avoided += 1
                    changed = True
        moved = set()
        for rid, destination in chosen.items():
            robot = self.couriers[rid]
            robot.position = destination
            if rid not in yield_moves:
                robot.route.pop(0)
            robot.battery -= self.cfg.energy_per_step
            robot.distance += 1
            self.total_distance += 1
            moved.add(rid)
            self.stall_ticks[rid] = 0
            if robot.route and robot.battery < self.cfg.energy_per_step:
                self.emergency_recovery(robot, 'battery exhausted after movement')
        for rid, robot in movers.items():
            if rid not in moved and robot.route:
                self.stall_ticks[rid] += 1
                if robot.battery < self.cfg.energy_per_step:
                    self.emergency_recovery(robot, 'battery depleted in transit')
                elif self.stall_ticks[rid] >= 35:
                    self.deadlocks += 1
                    self.emergency_recovery(robot, 'no progress for 35 ticks')
                    self.stall_ticks[rid] = 0
        for robot in self.couriers.values():
            if (robot.state not in moving_states or robot.route
                    or robot.position != robot.target):
                continue
            if robot.state == 'TO_PICKUP':
                order = self.orders[robot.order_id]
                order.picked_at = self.clock
                self.set_route(robot, order.spec.dropoff, 'TO_DROPOFF')
                self.event('pickup', robot=robot.id, order=order.spec.id)
            elif robot.state == 'TO_DROPOFF':
                order_id = robot.order_id
                order = self.orders[order_id]
                order.status = 'AWAIT_COMPLETE'
                robot.order_id = None
                robot.state = 'IDLE'
                robot.target = None
                self.send('TASK_COMPLETE', robot.id, 'D', order_id)
            elif robot.state == 'TO_CHARGER':
                robot.state = 'WAIT_CHARGE'
                self.send('CHARGE_REQUEST', robot.id, robot.charger_id)
    def assert_invariants(self):
        positions = [r.position for r in self.couriers.values()]
        assert len(positions) == len(set(positions)), 'couriers overlap'
        assert all(0 <= p[0] < self.grid.width and 0 <= p[1] < self.grid.height
                   and p not in self.scenario.walls for p in positions)
        assert all(0 <= r.battery <= r.capacity for r in self.couriers.values())
        active = [o.robot for o in self.orders.values()
                  if o.robot and o.status in ('IN_PROGRESS', 'RESERVED')]
        assert len(active) == len(set(active)), 'robot holds two orders'
        self.charger_constraint_checks += 1
        errors = []
        charging_by_station = defaultdict(list)
        for robot in self.couriers.values():
            if robot.state == 'CHARGING':
                charging_by_station[robot.charger_id].append(robot.id)
                station = self.chargers.get(robot.charger_id)
                if station is None or station.occupant != robot.id:
                    errors.append('charging robot without matching station occupant: ' + robot.id)
        for station in self.chargers.values():
            if station.occupant in station.queue:
                errors.append('occupant also queued: ' + station.id)
            if len(set(station.queue)) != len(station.queue):
                errors.append('duplicate station queue entries: ' + station.id)
            if set(station.queue) != set(station.queued_at) or set(station.queue) != set(station.battery_at_request):
                errors.append('station queue and timestamp records disagree: ' + station.id)
            if station.occupant is not None and station.occupant not in self.couriers:
                errors.append('unknown station occupant: ' + station.id)
            if len(charging_by_station[station.id]) > 1:
                errors.append('multiple charging robots at station: ' + station.id)
            if station.occupant is not None:
                occupant = self.couriers.get(station.occupant)
                if occupant is not None and occupant.state not in ('WAIT_CHARGE', 'CHARGING', 'IDLE'):
                    errors.append('station occupied by robot in incompatible state: ' + station.id)
        if errors:
            self.charger_violations += len(errors)
            raise AssertionError('; '.join(errors))
        for robot in self.couriers.values():
            if robot.order_id is not None:
                assert self.orders[robot.order_id].robot == robot.id
                assert self.orders[robot.order_id].status == 'IN_PROGRESS'
            if robot.pending_award is not None:
                assert self.orders[robot.pending_award].status == 'RESERVED'
    def step(self):
        self.deliver()
        self.broker_step()
        for robot in self.couriers.values():
            self.agent_step(robot)
        for station in self.chargers.values():
            self.station_step(station)
        self.move_step()
        self.assert_invariants()
        self.clock += 1
    def run(self):
        while self.clock < self.cfg.max_ticks:
            self.step()
            if self.completed == len(self.orders):
                break
        finished = [o for o in self.orders.values() if o.status == 'COMPLETED']
        latencies = [o.completed_at - o.spec.release for o in finished]
        assignment_times = [o.assigned_at - o.spec.release for o in finished]
        return {
            'scenario': self.cfg.scenario, 'seed': self.cfg.seed, 'mode': self.mode,
            'charge_policy': self.charge_policy,
            'width': self.cfg.width, 'height': self.cfg.height,
            'robots': len(self.couriers), 'stations': len(self.chargers),
            'orders': len(self.orders), 'completed': len(finished),
            'completion_rate': len(finished) / len(self.orders),
            'makespan': max((o.completed_at for o in finished), default=None),
            'mean_delivery_ticks': round(sum(latencies) / len(latencies), 3) if latencies else None,
            'mean_assignment_ticks': round(sum(assignment_times) / len(assignment_times), 3) if assignment_times else None,
            'messages': sum(self.message_counts.values()),
            'message_bytes': self.message_bytes,
            'coordination_messages': sum(count for key, count in self.message_counts.items()
                                         if key.startswith('TASK_') and key != 'TASK_COMPLETE'),
            'coordination_ticks': sum(assignment_times),
            'distance_cells': self.total_distance,
            'energy_used': self.total_distance * self.cfg.energy_per_step,
            'charging_wait_ticks': self.charge_wait_total,
            'charging_grants': self.charging_grants,
            'mean_queue_wait': round(sum(self.charging_waits)/len(self.charging_waits), 3) if self.charging_waits else None,
            'max_queue_wait': max(self.charging_waits, default=None),
            'urgent_mean_wait': round(sum(self.urgent_waits)/len(self.urgent_waits), 3) if self.urgent_waits else None,
            'urgent_grants': len(self.urgent_waits),
            'queue_wait_jain': round((sum(w+1 for w in self.charging_waits)**2) /
                                     (len(self.charging_waits)*sum((w+1)**2 for w in self.charging_waits)), 4)
                               if self.charging_waits else None,
            'max_charger_queue': self.max_charger_queue,
            'charging_busy_ticks': self.charging_busy_ticks,
            'charger_violations': self.charger_violations,
            'charger_constraint_checks': self.charger_constraint_checks,
            'pending_charge_requests': sum(len(c.queue) for c in self.chargers.values()),
            'pending_max_wait': max((self.clock - c.queued_at[rid]
                                      for c in self.chargers.values() for rid in c.queue), default=None),
            'collisions_prevented': self.collisions_avoided,
            'simulation_ticks': self.clock,
            'unfinished': len(self.orders) - len(finished),
            'emergency_recoveries': self.recoveries,
            'deadlock_recoveries': self.deadlocks,
            'recovery_service_ticks': self.recovery_ticks_total,
            'unresolved_in_transit': sum(r.state in ('TO_PICKUP', 'TO_DROPOFF', 'TO_CHARGER')
                                         and r.battery < self.cfg.energy_per_step
                                         for r in self.couriers.values()),
        }
