with open("new.html", "r") as input_file:
    with open("web_page.h", "w") as result_file:
        result_file.write("#define WEB_PAGE \"")
        for line in input_file:
            line = line.strip()
            line = line.replace("\"", "\'")
            line = line.replace("Temperature: 0", "Temperature: \" + String(Temperature) + \"")
            line = line.replace("Humidity: 0", "Humidity: \" + String(Humidity) + \"")
            result_file.write(line)
        result_file.write("\"")
