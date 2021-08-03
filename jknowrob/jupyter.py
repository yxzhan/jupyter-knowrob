import rospy
from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologNextSolutionResponse, PrologFinish
import json

"""SWI-Prolog kernel wrapper"""
from ipykernel.kernelbase import Kernel


def main():
    from ipykernel.kernelapp import IPKernelApp
    IPKernelApp.launch_instance(kernel_class=KnowrobKernel)


class KnowrobKernel(Kernel):
    implementation = 'KnowRob'
    implementation_version = '0.0.1'
    language = 'Prolog'
    language_version = '1.0'
    language_info = {'name': 'swipl',
                     'mimetype': 'text/plain'}
    banner = "KnowRob Kernel"

    def __init__(self, name_space='rosprolog', timeout=None, wait_for_services=True):
        """
        @type   name_space: str
        @param  timeout: Amount of time in seconds spend waiting for rosprolog to become available.
        @type   timeout: int
        """
        self.id = 0
        self._simple_query_srv = rospy.ServiceProxy(
            '{}/query'.format(name_space), PrologQuery)
        self._next_solution_srv = rospy.ServiceProxy(
            '{}/next_solution'.format(name_space), PrologNextSolution)
        self._finish_query_srv = rospy.ServiceProxy(
            '{}/finish'.format(name_space), PrologFinish)
        if wait_for_services:
            rospy.loginfo('waiting for {} services'.format(name_space))
            self._finish_query_srv.wait_for_service(timeout=timeout)
            self._simple_query_srv.wait_for_service(timeout=timeout)
            self._next_solution_srv.wait_for_service(timeout=timeout)
            rospy.loginfo('{} services ready'.format(name_space))

    def finish(self):
        if not self._finished:
            try:
                self._finish_query_srv(id=self.get_id())
            finally:
                self._finished = True

    def do_execute(self, code, silent,
                   store_history=True,
                   user_expressions=None,
                   allow_stdin=False):
        """This function is called when a code cell is
        executed."""
        if not silent:
            # We run the Prolog code and get the output.
            self.id += 1
            solutions = []
            result = self._simple_query_srv(id=str(self.id), query=code)
            try:
                while not self._finished:
                    next_solution = self._next_solution_srv(id=self.get_id())
                    if (next_solution.status == srv.PrologNextSolutionResponse.OK):
                        solutions.append(dict(json.loads(next_solution.solution)))
                    elif (next_solution.status == srv.PrologNextSolutionResponse.WRONG_ID 
                            or next_solution.status == srv.PrologNextSolutionResponse.QUERY_FAILED):
                        err_payload = {'ename': "",
                                        'evalue': "",
                                        'traceback':'Prolog query failed: {}'.format(next_solution.solution)}
                        self.send_response(self.iopub_socket,
                                   'error', err_payload)
                        break
                    elif (next_solution.status == srv.PrologNextSolutionResponse.NO_SOLUTION):
                        # We send back the result to the frontend.
                        stream_content = {'name': 'stdout',
                                            'text': "\n".join(solutions)}
                        self.send_response(self.iopub_socket,
                                            'stream', stream_content)
                        break
                    else:
                        raise PrologException('Unknown query status {}'.format(next_solution.status))
            finally:
                self.finish()
        return {'status': 'ok',
                # The base class increments the execution
                # count
                'execution_count': self.execution_count,
                'payload': [],
                'user_expressions': {},
               }