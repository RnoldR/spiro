import time
from lib_vsfsm import vsFSM

# Constants
# Distances in cm's
SP_SAFE = 60
SP_UNSAFE = 30

# Durations in seconds
DR_STOP = 1
DR_BACK = 0.5

# Create state machine and add states
fsm = vsFSM()
fsm.add_states(['Fast', 'Slow', 'Stop', 'Back', 'Turn', 'Halt'])

# For each state add transitions
fsm.add_transition('Fast', lambda inputs: inputs['Interrupt'], ['Halt'])
fsm.add_transition('Fast', lambda inputs: inputs['Distance'] >= SP_SAFE, ['Fast'])
fsm.add_transition('Fast', lambda inputs: inputs['Distance'] < SP_SAFE, ['Slow'])

fsm.add_transition('Slow', lambda inputs: inputs['Interrupt'], ['Halt'])
fsm.add_transition('Slow', lambda inputs: inputs['Distance'] >= SP_SAFE, ['Fast'])
fsm.add_transition('Slow', lambda inputs: SP_UNSAFE <= inputs['Distance'] < SP_SAFE, ['Slow'])
fsm.add_transition('Slow', lambda inputs: inputs['Distance'] < SP_UNSAFE, ['Stop'])

fsm.add_transition('Stop', lambda inputs: inputs['Interrupt'], ['Halt'])
fsm.add_transition('Stop', lambda inputs: time.time() - inputs['Timer'] < DR_STOP, ['Stop'])
fsm.add_transition('Stop', lambda inputs: time.time() - inputs['Timer'] >= DR_STOP, ['Back'])

fsm.add_transition('Back', lambda inputs: inputs['Interrupt'], ['Halt'])
fsm.add_transition('Back', lambda inputs: time.time() - inputs['Timer'] < DR_BACK, ['Back'])
fsm.add_transition('Back', lambda inputs: time.time() - inputs['Timer'] >= DR_BACK, ['Turn'])

fsm.add_transition('Turn', lambda inputs: inputs['Interrupt'], ['Halt'])
fsm.add_transition('Turn', lambda inputs: inputs['Distance'] < SP_UNSAFE, ['Turn'])
fsm.add_transition('Turn', lambda inputs: inputs['Distance'] >= SP_UNSAFE, ['Slow'])

fsm.add_transition('Halt', lambda inputs: inputs['Interrupt'], ['Halt'])

# Add input values
fsm.set_input('Distance', 20)
fsm.set_input('Timer', time.time())
fsm.set_input('Interrupt', False)

print('Start state: ', fsm.start_state)

halter = time.time()
current_state = fsm.start_state
while current_state != 'Halt':
    print('Current state:', current_state)
    new_state = fsm.evaluate(current_state)
    if new_state != current_state:
        print('New state:', new_state)
        fsm.set_input('Timer', time.time())
        current_state = new_state

    time.sleep(0.24)

    if time.time() - halter > 3:
        fsm.set_input('Interrupt', True)

