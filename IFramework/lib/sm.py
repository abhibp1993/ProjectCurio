"""
File: sm.py (State Machines)
Project: ProjectCurio

Author: Abhishek N. Kulkarni
Acknowledgements: Prof. Milind Patwardhan, Prof. Mrunal Shidore, Prof. Milind Kamble,
    Monica Patel, Aditya Joshi, Shruti Phadke.

Repo: https://github.com/abhibp1993/BCUDProject 

Description:
    Defines basic structure and primitives for laying layers of abstractions
    for the complete project
    
    Classes: 
    ** StateMachine: BASE --> DO NOT USE DIRECTLY
    - Gain 
    - Wire
    - Delay or R
    - Integrator
    - Derivative
    - Adder
    - Subtractor
    - Cascade
    - Parallel
    - Parallel2
    - Feedback
    - FeedbackAdd
    - FeedbackSubtract 


****************************************************************************
License: 

This file is part of ProjectCurio.

ProjectCurio is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

ProjectCurio is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with ProjectCurio.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************************

"""

"""
File: StateMachine.py
Author: Abhishek N. Kulkarni


    
To be done:
    1. Implement Run() function - find out why and how to use this!
"""

import util


#===========================================================================
############### STATE MACHINE: BASE CLASSES ################################
#===========================================================================

class StateMachine(object):
    """
    Represents the basic StateMachine. The class provides a base for implementation of 
    all state machines. This class MUST NOT be instantiated but classes may be derived
    out of this class. 

    Global Variables:
        startState: a generic start state of stateMachine, default: None
        
    """
    startState = None
    doneCondition = None

    def initialize(self):
        """
        Initializes the machine with currState variable to startState.
        [Optionally, when debugger is introduced, it must reset the logging related lists.]
        """
        self.currState = self.startState


    def getNextValues(self, state, inp):
        """
        Returns the output which should result IF the given input is applied to the machine
        IF it were in given state. In other words, returns (but not performs) the transition
        on state machine.

        @state: valid state of machine
        @inp: input provided to machine
        
        @return: 2-tuple of (newState, output)
        [For more complete description, refer documentation]
        """
        raise NotImplementedError("getNextValues method is not implemented")

    def step(self, inp):
        """
        Advances the machine my one step. Applies the argument - inp - to current state.
        The resultant state is the new state of machine. 
        [Internal function. Do not make use of it/tweak it.]

        @inp: input to machine
        @return: the output of the transition performed on the machine.

        Errors:
            1. ValueError: Too many values to unpack - If getNextValues does not return a 2-tuple
            2. ValueError: Too less values to unpack - If getNextValues does not return a 2-tuple

        """
        nState, output = self.getNextValues(self.currState, inp)
        self.currState = nState
        return output

    def transduce(self, inpSequence = list()):
        """
        Advances machine by applying inpSequence as input to machine. 
        Index - 0 being the first applied input.

        @inpSequence: ordered list of n-inputs
        @returns: ordered list of n-outputs resulting from n-transitions

        Errors: 
            1. ValueError: Empty Input Sequence

        """
        
        lstOutputs = list() 

        # Initialize the state machine
        self.initialize()

        # Check if inpSequence is non-empty
        if len(inpSequence) == 0:
            raise ValueError("Empty Input Sequence")

        # Loop and apply the inputs
        for i in inpSequence:
            try:
                lstOutputs.append(self.step(i))
            except Exception, ex:
                lstOutputs.append(None)
                print 'Step function failed, for input: ', i, ' ', str(ex)

        return lstOutputs

    def transduceFcn(self, inpFcn, nSteps = []):
        """
        Advances machine by calling input function with next-index of loop. 
        
        @inpFcn: a procedure with 1 positive-integer parameter
        @nSteps: list of +ve-integer indices which should be the inputs applied. [Eg. nSteps = range(1, 10)]
        @returns: ordered list of n-outputs resulting from n-transitions

        Errors: 
            1. ValueError: Empty Input Sequence

        """
        lstOutputs = list() 
        # Loop and apply the inputs
        for i in nSteps:
            try:
                lstOutputs.append(self.step(inpFcn(i)))
            except ex:
                lstOutputs.append(None)
                print 'Step function failed, for index: ', i, ' ', str(ex)

        return lstOutputs

    def done(self):
        """
        Evaluates if the machine is "done" it's working -- i.e. Is the Target Reached?

        @returns: True if doneCondition evaluates to true.
        [ doneCondition needs to be initialized using sm.initialize(condition) ]

        """
        try:
            if doneCondition(self.currState): return True
        except:
            # Add to log
            print "WARNING: doneCondition looks to have error OR is not initialized"

        return False

    def run(self):
        ### figure our what and how to choose inputs??

        # nState = self.startState
        # while not self.done(self.currState):
        #    output = self.step(???inp???)
        #    print output 

        raise NotImplementedError

    def __str__(self):
        try:
            return 'State Machine: Current State', str(self.currState)
        except:
            return 'State Machine: Not yet initialized'



#===========================================================================
############### STATE MACHINE: PRIMITIVES ##################################
#===========================================================================

class Gain(SM):
    """
    Represents a machine, whose output is scaled input by factor 'k'.
    The state is None.
    """
    def __init__(self, k = 1.0):
        """
        Instantiation.
        @k: gain of machine, float or integer or util.Complex instance
        """
        assert isinstance(k, (float, int, complex)), 'Gain Machine Instantiation Error: Gain must be int, float or complex'

        self.k = k
        self.startState = 0.0

    def getNextValues(self, state, inp):
        if inp == 'undefined':
            return ('undefined', 'undefined')
        else:
            return (self.k * inp, self.k * inp)

    def __str__(self):
        return 'Gain machine: K = ' + str(self.k)

class Wire(SM):
    """
    Represents a wire, a machine whose output is same as input. 
    """
    def __init__(self):
        self.intGainMachine = Gain(1.0)

    def getNextValues(self, state, inp):
        state, out = self.intGainMachine.getNextValues(state, inp)
        return (out, out)

    def __str__(self):
        return 'Wire machine, implemented with Gain machine'

class Delay(SM):
    """
    Represents a wire, a machine whose output is same as input at last time step.
    Input at -1th step is assumed to be 0.0 -> this is justified for all LTI-
    systems where system is assumed to start from rest.
    """
    def __init__(self):
        self.startState = 0.0

    def getNextValues(self, state, inp):
        return (inp, state)

    def __str__(self):
        return 'Delay machine'

class Integrator(SM):
    """
    Represents the accumulator machine -> output is addition of all past inputs
    """
    def __init__(self):
        self.startState = 0.0

    def getNextValues(self, state, inp):
        if inp == 'undefined':
            return ('undefined', 'undefined')
        else:
            return (state + inp, state + inp)

    def __str__(self):
        return 'Integrator machine'
    
class Derivative(SM):
    """
    Represents the machine whose output is difference of current and past input.
    """
    def __init__(self):
        self.startState = 0.0

    def getNextValues(self, state, inp):
        if inp == 'undefined':
            return ('undefined', 'undefined')
        else:
            return (inp, inp - state)

    def __str__(self):
        return 'Derivative machine'

class Adder(SM):
    """
    Represents a state machine, whose output is addition of inputs.
    
    Input must be a 2-tuple. 
    Output is (None, <addition>) tuple.
    """
    def safeAdd(self, inp1, inp2):
        if inp1 == 'undefined' or inp2 == 'undefined':
            return 'undefined'
        else:
            return inp1 + inp2

    def getNextValues(self, state, inp):
        inp1, inp2 = inp
        return (None, self.safeAdd(inp1, inp2))

class Subtractor(SM):
    """
    Represents a state machine, whose output is difference of inputs.
    
    Input must be a 2-tuple. 
    Output is (None, <addition>) tuple.
    """
    def safeDiff(self, inp1, inp2):
        if inp1 == 'undefined' or inp2 == 'undefined':
            return 'undefined'
        else:
            return inp1 - inp2

    def getNextValues(self, state, inp):
        inp1, inp2 = inp
        return (None, self.safeDiff(inp1, inp2))


# R can be another name for delay!
R = Delay   



#===========================================================================
############### STATE MACHINE: COMBINATORS #################################
#===========================================================================

class Cascade(SM):
    """
    Represents the cascade of 2 machines. 
    Constraints: Output of 1st machine MUST be EQUAL to Input of 2nd.
    State of Cascade: (state_1, state_2)
    Output of Cascade: Output of machine 2
    """
    def __init__(self, sm1, sm2):
        self.sm1 = sm1
        self.sm2 = sm2
        self.startState = (sm1.startState, sm2.startState)

    def getNextValues(self, state, inp):
        st1, st2 = state
        nState1, out1 = self.sm1.getNextValues(st1, inp)
        nState2, out2 = self.sm2.getNextValues(st2, out1)
        return ((nState1, nState2), out2)

    def __str__(self):
        return 'yet to be set'

class Parallel(SM):
    """
    Represents two state machines in parallel combination sharing same output.
    Constraints: Input vocabulary of both machines must be same. However, output
    vocabularies may be different. 

    Output of Parallel Combinator: 2-tuple of outputs from sm1, sm2.
    """
    def __init__(self, sm1, sm2):
        self.sm1 = sm1
        self.sm2 = sm2
        self.startState = (sm1.startState, sm2.startState)

    def getNextValues(self, state, inp):
        state1, state2 = state
        nState1, out1 = self.sm1.getNextValues(state1, inp)
        nState2, out2 = self.sm2.getNextValues(state2, inp)

        return ((nState1, nState2), (out1, out2))

class Parallel2(SM):
    """
    Represents two state machines in parallel combination sharing same input.
    Constraints: Input vocabulary of both machines must be same. However, output
    vocabularies may be different. 

    Output of Parallel Combinator: 2-tuple of outputs from sm1, sm2.
    """
    def __init__(self, sm1, sm2):
        self.sm1 = sm1
        self.sm2 = sm2
        self.startState = (sm1.startState, sm2.startState)

    def getNextValues(self, state, inp):
        state1, state2 = state
        inp1, inp2 = inp
        nState1, out1 = self.sm1.getNextValues(state1, inp1)
        nState2, out2 = self.sm2.getNextValues(state2, inp2)

        return ((nState1, nState2), (out1, out2))

class ParallelAdd(SM):
    """
    Represents a machine whose output is addition of outputs of each machine
    in parallel. The inputs can be either a 2-tuple (independent inputs) or
    single value(common inputs). 

    The outputs of machines must have valid definition of '+' operator.
    """
    def __init__(self, sm1, sm2):
        self.sm1 = sm1
        self.sm2 = sm2
        self.smAdd = Adder()
        self.startState = (sm1.startState, sm2.startState)

    def getNextValues(self, state, inp):

        if isinstance(inp, tuple):
            if len(inp) == 2:
                # out will be a 2-tuple from any parallel or parallel2 machine.
                nState, out = Parallel2(self.sm1, self.sm2).getNextValues(state, inp)
        else:
            nState, out = Parallel(self.sm1, self.sm2).getNextValues(state, inp)

        ignore, out = self.smAdd.getNextValues(None, out)
        return (nState, out)


class Feedback(SM):
    """
    Represents a feedback combination of single machine, where the output is fedback 
    to input directly. Note, feedback machine has no external inputs.
    """
    def __init__(self, m):
        self.sm = m
        self.startState = m.startState

    def getNextValues(self, state, inp):
        ignore, out = self.sm.getNextValues(state, 'undefined')
        nState, ignore = self.sm.getNextValues(state, out)
        return (nState, out)

class FeedbackAdd(SM):
    """
    Represents a feedback combination of 2 state machines, such that the feedback is
    added to input. (Generally a positive-feedback system will be achieved)

    A point to be noted:
        The issue of time-delay in feedback path gives rise to trouble. A remedy is to 
    step the feedback path with 'undefined' as input. This churns out the internal value
    of feedback path and allows to avoid inherent delay.
    [Note: Check documentation for more elaborate explaination of this code.]
    """
    def __init__(self, sm1, sm2):
        """
        Instantiation.

        @sm1: State machine derivative
        @sm2: State machine derivative
        """
        assert isinstance(sm1, SM), 'input machines must be derivatives of state machine class.'
        assert isinstance(sm2, SM), 'input machines must be derivatives of state machine class.'

        self.sm1 = sm1
        self.sm2 = sm2
        self.smAdd = Adder()
        self.startState = (self.sm1.startState, self.sm2.startState)

    def getNextValues(self, state, inp):
        st1, st2 = state
        nState2, out2 = self.sm2.getNextValues(st2, 'undefined')
        ign, err = self.smAdd.getNextValues(None, (out2, inp))
        nState1, out1 = self.sm1.getNextValues(st1, err)
        nState2, out2 = self.sm2.getNextValues(st2, out1)
        return ((nState1, nState2), out1)

    def __str__(self):
        return 'FeedbackSubtract machine: Machine1: ' + str(self.sm1) + 'Machine2: ' + str(self.sm2)

class FeedbackSubtract(SM):
    """
    Represents a feedback combination of 2 state machines, such that the feedback is
    subtracted from input. (Generally a negative-feedback system will be achieved)

    A point to be noted:
        The issue of time-delay in feedback path gives rise to trouble. A remedy is to 
    step the feedback path with 'undefined' as input. This churns out the internal value
    of feedback path and allows to avoid inherent delay.
    [Note: Check documentation for more elaborate explaination of this code.]
    """
    def __init__(self, sm1, sm2):
        """
        Instantiation.

        @sm1: State machine derivative
        @sm2: State machine derivative
        """
        assert isinstance(sm1, SM), 'input machines must be derivatives of state machine class.'
        assert isinstance(sm2, SM), 'input machines must be derivatives of state machine class.'

        self.sm1 = sm1
        self.sm2 = sm2
        self.smSub = Subtractor()
        self.startState = (self.sm1.startState, self.sm2.startState)

    def getNextValues(self, state, inp):
        
        # Uncouple the state and perform a 'hypothetical' transition on feedback path.
        st1, st2 = state            
        nState2, out2 = self.sm2.getNextValues(st2, 'undefined')
        
        # Subtract the output of feedback path from current input
        ign, err = self.smSub.getNextValues(None, (inp, out2))

        # Identify the actual transition on the machines.
        nState1, out1 = self.sm1.getNextValues(st1, err)
        nState2, out2 = self.sm2.getNextValues(st2, out1)

        return ((nState1, nState2), out1)

    def __str__(self):
        return 'FeedbackSubtract machine: Machine1: ' + str(self.sm1) + 'Machine2: ' + str(self.sm2)
   

#===========================================================================
############### STATE MACHINE: FILTERS #####################################
#===========================================================================

class SMA2(SM):
    def __init__(self):
        self.startState = util.Queue(2)
        self.startState.initialize()
      
    def getNextValues(self, state, inp):
        state.add(inp)      #note state is a util.Queue, add returns last poped element.
        return(state, float(sum(state))/len(state))

class SMA3(SM):
    def __init__(self):
        self.startState = util.Queue(3)
        self.startState.initialize()
      
    def getNextValues(self, state, inp):
        state.add(inp)      #note state is a util.Queue, add returns last poped element.
        return(state, float(sum(state))/len(state))

class SMA(SM):
    '''
    Generalized k-sample SMA Filter.

    We implement SMA filters using queue data structure for convenience. The queue is limited
    capacity queue, where the recent k-inputs are stored using FIFO manner. Refer to Queue 
    implementation documentation in util file for more information on how Queue behaves.
    The point is, such an implementation of Queue abstracts gory details and allows a slick code
    to be implemented for SMA filters!
    '''
    def __init__(self, k):
        self.k = k
        self.startState = util.Queue(self.k)
        self.startState.initialize()
      
    def getNextValues(self, state, inp):
        state.add(inp)      #note state is a util.Queue, add returns last poped element.
        return(state, float(sum(state))/len(state))



#===========================================================================
############### STATE MACHINE: CONTROLLERS #################################
#===========================================================================

class PID(SM):
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.machine = ParallelAdd(Gain(Kp), \
                       ParallelAdd(Cascade(Gain(Ki), Integrator()), \
                                   Cascade(Gain(Kd), Derivative())))
    
        self.startState = self.machine.startState


    def getNextValues(self, state, inp):
        return self.machine.getNextValues(state, inp)
