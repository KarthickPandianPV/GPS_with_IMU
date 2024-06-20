#include "datalogger.hpp"

namespace gps_with_imu {
Datalogger::Datalogger(/* args */) {}
Datalogger::~Datalogger() {}

void Datalogger::initialize() {
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(date_string, 30, "%Y-%m-%d", timeinfo);
  strftime(time_string, 30, "%H-%M-%S", timeinfo);
  char filename[50];
  strcpy(filename, date_string);
  strcat(filename, "_");
  strcat(filename, time_string);
  strcat(filename, type);
  workbook = workbook_new(filename);
  worksheet = workbook_add_worksheet(workbook, "Data");
  format_bold = workbook_add_format(workbook);
  format_txt_clr = workbook_add_format(workbook);
  format_set_bold(format_bold);
  format_set_font_color(format_txt_clr, LXW_COLOR_BLUE);
  worksheet_write_string(worksheet, 0, 0, "Time", format_bold);
  worksheet_write_string(worksheet, 0, 1, "Magnetic Field in X", format_bold);
  worksheet_write_string(worksheet, 0, 2, "Magnetic Field in Y", format_bold);
  worksheet_write_string(worksheet, 0, 3, "Magnetic Field in Z", format_bold);
  worksheet_write_string(worksheet, 0, 4, "Acceleration in X", format_bold);
  worksheet_write_string(worksheet, 0, 5, "Acceleration in Y", format_bold);
  worksheet_write_string(worksheet, 0, 6, "Acceleration in Z", format_bold);
  worksheet_write_string(worksheet, 0, 7, "Velocity in X", format_bold);
  worksheet_write_string(worksheet, 0, 8, "Velocity in Y", format_bold);
  worksheet_write_string(worksheet, 0, 9, "Velocity in Z", format_bold);
  worksheet_write_string(worksheet, 0, 10, "Distance in X", format_bold);
  worksheet_write_string(worksheet, 0, 11, "Distance in Y", format_bold);
  worksheet_write_string(worksheet, 0, 12, "Distance in Z", format_bold);
}

void Datalogger::write_data(vec3f magnetic_field, vec3f acceleration,
                            vec3f velocity, vec3f distance) {
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(time_string, sizeof(time_string), "%H-%M-%S", timeinfo);
  worksheet_write_string(worksheet, row, 0, time_string, format_txt_clr);
  worksheet_write_number(worksheet, row, 1, magnetic_field.x, NULL);
  worksheet_write_number(worksheet, row, 2, magnetic_field.y, NULL);
  worksheet_write_number(worksheet, row, 3, magnetic_field.z, NULL);
  worksheet_write_number(worksheet, row, 4, acceleration.x, NULL);
  worksheet_write_number(worksheet, row, 5, acceleration.y, NULL);
  worksheet_write_number(worksheet, row, 6, acceleration.z, NULL);
  worksheet_write_number(worksheet, row, 7, velocity.x, NULL);
  worksheet_write_number(worksheet, row, 8, velocity.y, NULL);
  worksheet_write_number(worksheet, row, 9, velocity.z, NULL);
  worksheet_write_number(worksheet, row, 10, distance.x, NULL);
  worksheet_write_number(worksheet, row, 11, distance.y, NULL);
  worksheet_write_number(worksheet, row, 12, distance.z, NULL);

  row++;
}

}  // namespace gps_with_imu