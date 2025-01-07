import time

# State descriptions
class vsFSM():
    def __init__(self):
        self.states = {}
        self.inputs = {}
        self.start_state = ''

        return
    
    def add_state(self, state_name: str):
        self.states[state_name] = {}

        if len(self.start_state) == 0:
            self.start_state = state_name

        return
    
    def add_states(self, state_names: list):
        if len(state_names) > 0:
            for state_name in state_names:
                self.add_state(state_name)

        return
    
    def add_transition(self,
                       state_name: str, 
                       condition: any, 
                       output: list,
                      ):
        
        if state_name not in self.states.keys():
            raise ValueError(f'State not yet added: {state_name}')
        
        # generate unique state name
        i = 1
        while i in self.states[state_name].keys():
            i += 1

        # create transition dictionary
        transition = {'condition': condition,
                      'output': output}
        
        # add it to the state
        self.states[state_name][i] = transition

        return
    
    def set_start_state(self, state_name: str):
        self.start_name = state_name

        return
    
    def set_input(self, key: str, value):
        self.inputs[key] = value

        return
    
    def evaluate(self, current: str):
        state = self.states[current]

        # iterate thru all transitions
        for transition in state.keys():
            # Find and evaluate the condition with the inputs
            func = state[transition]['condition']
            result = func(self.inputs)

            # if result is True return first argument from list
            if result:
                return state[transition]['output'][0]

            # if

        # for

        return None
    
    ### evaluate ###

### evaluate ###

