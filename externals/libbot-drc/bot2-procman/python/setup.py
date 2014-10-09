from distutils.core import setup

setup(name="lcm", version="0.1.0",
      package_dir = { '' : 'src' },
      packages=["bot_procman", "bot_procman/sheriff_gtk"],
      scripts=["scripts/bot-procman-sheriff"])
