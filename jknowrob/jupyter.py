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

        self.unique_id = ''.join(random.choice(string.ascii_lowercase) for i in range(10))
        rospy.init_node('jupyter_knowrob' + '_' + self.unique_id)

        self.name_space='rosprolog'
        self.timeout=None
        self.wait_for_services=True

        self._finished = False
        self.id_prefix = self.unique_id + '_'
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
            self.init_query()


    def get_id(self):
        return self.id_prefix + str(self.id)


    def finish(self, finished_id):
        if not self._finished:
            try:
                self._finish_query_srv(id=finished_id)
            finally:
                self._finished = True


    def init_query(self):
        self.id += 1
        current_id = self.get_id()
        q = 'register_ros_package(knowrob_cloud), findall([_X, _Y], rdf_current_ns(_X, _Y), NS)'
        self._simple_query_srv(id=current_id, query=q)
        solution = json.loads(self._next_solution_srv(id=current_id).solution)
        self.log.warn('loaded namespaces')
        for k, v in solution["NS"]:
            self.ns_dict[v] = k + ':'


    def send_response_ok(self, output):
        for ns, short in self.ns_dict.items():
            output = output.replace(ns, short)
        stream_content = {'name': 'stdout', 'text': output}
        self.send_response(self.iopub_socket, 'stream', stream_content)


    def create_query(self, code):
        # TODO: At additional analsis and sanytizing
        if ":-" in code:
            return "cloud_consult_string(" + self.get_id() + ",\"" + code + "\")"
        return code


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
            current_id = self.get_id()
            solutions = []
            self._finished = False
            # Create a query out of the input and send it to rosprolog
            query = self.create_query(code)
            result = self._simple_query_srv(id=current_id, query=query)
            # Create the answer
            while not self._finished:
                next_solution = self._next_solution_srv(id=current_id)
                if (next_solution.status == PrologNextSolutionResponse.OK):
                    # Collect the next solution
                    solution = dict(json.loads(next_solution.solution))
                    #If the solution is a empty dict, the answer is true
                    if (solution == dict()):
                        self.send_response_ok('true')
                        break
                    solutions.append(dict(json.loads(next_solution.solution)))
                elif (next_solution.status == PrologNextSolutionResponse.WRONG_ID 
                        or next_solution.status == PrologNextSolutionResponse.QUERY_FAILED):
                    # Send error message
                    err_payload = {'ename': "Prolog Error",
                                    'evalue': "Prolog query failed",
                                    'traceback':['Prolog query failed: {}'.format(str(next_solution.solution))]}
                    self.send_response(self.iopub_socket,
                               'error', err_payload)
                    self.log.error(str(next_solution.solution), exc_info=True)
                    self.log.error('Prolog query failed: {}'.format(str(next_solution.solution)), exc_info=True)
                    break
                elif (next_solution.status == PrologNextSolutionResponse.NO_SOLUTION):
                    # If no additional solution are found send the response
                    output = 'false'
                    if (len(solutions) > 0):
                        output = ';\n'.join([',\n'.join(['{}: {}'.format(k, v) for k, v in solDict.items()]) for solDict in solutions])
                    self.send_response_ok(output)
                    break
                else:
                    self.log.error('Unknown query status {}'.format(next_solution.status))
            self.finish(current_id)
        return {'status': 'ok',
                # The base class increments the execution
                # count
                'execution_count': self.execution_count,
                'payload': [],
                'user_expressions': {},
               }