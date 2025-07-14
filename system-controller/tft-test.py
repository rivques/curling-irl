import belay


controller = belay.Device('/dev/tty.usbmodem14101')

controller.sync("./lib")

@controller.setup
def controller_setup():
    from machine import Pin, ADC, SPI
    import gc9a01py as gc9a01
    import NotoSansMono as font
    slider = ADC(Pin(26))
    button = Pin(0, Pin.IN, Pin.PULL_UP)
    spi = SPI(0, baudrate=6000000, sck=Pin(2), mosi=Pin(3))
    tft = gc9a01.GC9A01(
        spi,
        dc=Pin(28, Pin.OUT),
        cs=Pin(1, Pin.OUT),
        reset=Pin(29, Pin.OUT),
        rotation=0)

@controller.task
def show_scores(player1_score, player2_score, player1_going, round_number, max_rounds):
    tft.fill(gc9a01.color565(0, 0, 0))  # Clear the screen
    tft.write(font, f"Player 1: {player1_score}", 20, 10, gc9a01.color565(255, 255, 255))
    tft.write(font, f"Player 2: {player2_score}", 10, 30, gc9a01.color565(255, 255, 255))
    tft.write(font, f"Round: {round_number}/{max_rounds}", 10, 50, gc9a01.color565(255, 255, 255))
    tft.write(font, f"Player {'1' if player1_going else '2'}'s turn", 10, 70, gc9a01.color565(255, 255, 255))

controller_setup()
show_scores(0, 0, True, 1, 10)  # Initial call to display scores
while True:
    pass  # Keep the script running to maintain the display