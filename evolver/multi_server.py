import asyncio
from aiohttp import web
from threading import Thread

class MultiServer:

    def __init__(self, loop=None):
        self._apps = []
        self.user_supplied_loop = loop is not None
        if loop is None:
            self.loop = asyncio.get_event_loop()
        else:
            self.loop = loop

    def add_app(self, port):
        app = web.Application(loop=self.loop)

        self._apps.append((app, port))

        return app

    @staticmethod
    async def shutdown(app):
        for ws in app['websockets'].values():
            await ws.close()
        app['websockets'].clear()

    async def start_site(self, app):
        address='0.0.0.0'
        runner = web.AppRunner(app[0])
        await runner.setup()
        site = web.TCPSite(runner, address, app[1])
        await site.start()

    def run_all(self):
        app_num = 0
        try:
            for app in self._apps:
                app[0]['websockets'] = {}
                app[0].on_shutdown.append(MultiServer.shutdown)

                self.loop.create_task(self.start_site(app))

                print('Running app {0}'. format(app_num))
                app_num += 1
            t = Thread(target = start_background_loop, args = (self.loop,))
            t.daemon = True
            t.start()

        except KeyboardInterrupt:
            print('Exiting application due to KeyboardInterrupt')

def start_background_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()
