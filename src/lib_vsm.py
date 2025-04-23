import time

# State descriptions
class vsM():
    def __init__(self):
        self.states = {}
        self.inputs = {}
        self.outputs = {}
        self.start_state = ''
        self.current_state = ''
        self.state_changes = True

        return
    
    ### __init__ ###
    
    
    def add_state(self, state_name: str):
        self.states[state_name] = {}

        if len(self.start_state) == 0:
            self.start_state = state_name

        return
    
    ### add_state ###
    
    
    def add_states(self, state_names: list):
        if len(state_names) > 0:
            for state_name in state_names:
                self.add_state(state_name)

        return
    
    ### add_states ###
    
    
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
    
    ### add_transition ###
    
    
    def set_start_state(self, state: str):
        
        if state in self.states.keys():
            self.start_state = state
            
        else:
            raise ValueError('Trying to set start_state to a non-existant values: ' + state);
        
        # if

        return
    
    ### set_start_state ###
    
    
    def get_start_state(self):
    
        return self.start_state
    
    ### get_stater_state ###
    

    def set_current_state(self, state):

        if state in self.states.keys():
            self.state_changed = state != self.current_state
            self.current_state = state
            
        else:
            raise ValueError('Trying to set current_state to a non-existant values: ' + state);
        
        # if
    
        return
    
    ### set_current_state ###
    
    
    def get_current_state(self):
        
        return self.current_state
    
    ### get_current_state ###


    def set_input(self, key: str, value):
       
        self.inputs[key] = value

        return
    
    ### set_input ###
    
    
    def evaluate(self):
        state = self.states[self.current_state]
   
        # iterate thru all transitions
        for transition in state.keys():
            # Find and evaluate the condition with the inputs
            func = state[transition]['condition']
            result = func(self.inputs)

            # if result is True return first argument from list
            if result:
                self.set_current_state(state[transition]['output'][0])

            # if

        # for

        return None
    
    ### evaluate ###

### Class: vsM ###
    
    

