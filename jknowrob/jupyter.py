import string
import random
import rospy
from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologNextSolutionResponse, PrologFinish
import json
from ipykernel.ipkernel import IPythonKernel


def main():
    from ipykernel.kernelapp import IPKernelApp
    IPKernelApp.launch_instance(kernel_class=KnowrobKernel)


class KnowrobKernel(IPythonKernel):
    implementation = 'KnowRob'
    implementation_version = '0.0.1'
    language = 'Prolog'
    language_version = '1.0'
    language_info = {'name': 'swipl',
                     'mimetype': 'text/plain'}
    banner = "KnowRob Kernel"

    def __init__(self, **kwargs):
        """
        @type   name_space: str
        @param  timeout: Amount of time in seconds spend waiting for rosprolog to become available.
        @type   timeout: int
        """
        IPythonKernel.__init__(self, **kwargs)

        rospy.init_node('jupyter_knowrob')

        self.name_space='rosprolog'
        self.timeout=None
        self.wait_for_services=True

        self._finished = False
        self.id_prefix = (''.join(random.choice(string.ascii_lowercase) for i in range(10))) + '_'
        self.id = 0

        self.ns_dict = dict()

        self._simple_query_srv = rospy.ServiceProxy(
            '{}/query'.format(self.name_space), PrologQuery)
        self._next_solution_srv = rospy.ServiceProxy(
            '{}/next_solution'.format(self.name_space), PrologNextSolution)
        self._finish_query_srv = rospy.ServiceProxy(
            '{}/finish'.format(self.name_space), PrologFinish)
        if self.wait_for_services:
            self.log.warn('waiting for {} services'.format(self.name_space))
            self._finish_query_srv.wait_for_service(timeout=self.timeout)
            self._simple_query_srv.wait_for_service(timeout=self.timeout)
            self._next_solution_srv.wait_for_service(timeout=self.timeout)
            self.log.warn('{} services ready'.format(self.name_space))
            self.load_namespace()


    def get_id(self):
        return self.id_prefix + str(self.id)


    def finish(self):
        if not self._finished:
            try:
                self._finish_query_srv(id=str(self.get_id()))
            finally:
                self._finished = True


    def load_namespace(self):
        self.id += 1
        q = 'findall([_X, _Y], rdf_current_ns(_X, _Y), NS)'
        self._simple_query_srv(id=self.get_id(), query=q)
        solution = json.loads(self._next_solution_srv(id=self.get_id()).solution)
        self.log.warn('loaded namespaces')
        for k, v in solution["NS"]:
            self.ns_dict[v] = k + ':'


    def send_response_ok(self, output):
        for ns, short in self.ns_dict.items():
            output = output.replace(ns, short)
        stream_content = {'name': 'stdout', 'text': output}
        self.send_response(self.iopub_socket, 'stream', stream_content)


    def do_execute(self, code, silent,
                   store_history=True,
                   user_expressions=None,
                   allow_stdin=False):
        """This function is called when a code cell is
        executed."""
        self.log.warn("Run query", exc_info=True)
        if not silent:
            # We run the Prolog code and get the output.
            self.id += 1
            solutions = []
            self._finished = False
            result = self._simple_query_srv(id=str(self.get_id()), query=code)
            try:
                while not self._finished:
                    next_solution = self._next_solution_srv(id=str(self.get_id()))
                    if (next_solution.status == PrologNextSolutionResponse.OK):
                        solution = dict(json.loads(next_solution.solution))
                        if (solution == dict()):
                            self.send_response_ok('true')
                            break
                        solutions.append(dict(json.loads(next_solution.solution)))
                    elif (next_solution.status == PrologNextSolutionResponse.WRONG_ID 
                            or next_solution.status == PrologNextSolutionResponse.QUERY_FAILED):
                        err_payload = {'ename': "",
                                        'evalue': "",
                                        'traceback':'Prolog query failed: {}'.format(next_solution.solution)}
                        self.send_response(self.iopub_socket,
                                   'error', err_payload)
                        self.log.error("Query failed", exc_info=True)
                        break
                    elif (next_solution.status == PrologNextSolutionResponse.NO_SOLUTION):
                        # We send back the result to the frontend.
                        output = 'false'
                        if (len(solutions) > 0):
                            output = ';\n'.join([',\n'.join(['{}: {}'.format(k, v) for k, v in solDict.items()]) for solDict in solutions])
                        self.send_response_ok(output)
                        break
                    else:
                        self.log.error('Unknown query status {}'.format(next_solution.status))
            finally:
                self.finish()
        return {'status': 'ok',
                # The base class increments the execution
                # count
                'execution_count': self.execution_count,
                'payload': [],
                'user_expressions': {},
               }